"""Single owner thread for serial/CAN and UDP. GUI only queues requests."""
import threading, queue, socket, time, math
from control import Sequencer, Settings, parse_vr5

class Backend(threading.Thread):
    def __init__(self, events):
        super().__init__(daemon=False); self.events=events; self.commands=queue.Queue()
        self.stop_requested=threading.Event(); self.quit_requested=threading.Event()
        self.can=None; self.motor=None; self.udp=None; self.fb=None; self.fb_time=0
        self.seq=Sequencer(); self.pending=False; self.armed=False; self.settings=Settings()
        self.last_cycle=time.monotonic(); self.last_send_time=time.monotonic(); self.power_ready_at=0; self.mode="real"
    def request(self,kind,**kwargs): self.commands.put((kind,kwargs))
    def emit(self,kind,**kwargs): self.events.put((kind,kwargs))
    def reply(self,msg):
        if self.udp: self.udp.sendto(msg.encode('ascii'),('127.0.0.1',5051))
        self.emit('log',message='TX '+msg)
    def abort(self,reason):
        self.seq.stop(reason)
        if self.pending: self.reply('GRIPPER_ABORTED')
        self.pending=False; self.emit('log',message=reason+'; no more servo frames (heartbeat stop ~500 ms per existing driver)')
    def disconnect(self):
        self.abort('Disconnect'); self.armed=False
        if self.can: self.can.close()
        self.can=self.motor=self.fb=None; self.emit('connection',connected=False)
    def connect(self,port,motor_id,mode="real"):
        if self.can: raise RuntimeError('Disconnect first')
        from ec_a4310_waveshare_test import WaveshareUsbCanA, EC_A4310
        self.mode=mode
        if mode=="simulation":
            from simulation import SimCan, SimMotor
            self.can=SimCan(); EC_A4310=SimMotor
        else:
            self.can=WaveshareUsbCanA(port,timeout=.005)
        try:
            self.can.configure_1m_standard_normal(); self.motor=EC_A4310(self.can,motor_id)
            if self.motor.motor_id is None: self.motor.discover_id()
            ok=self.motor.set_qna_mode()
            if not ok: self.emit('log',message='Q&A mode has no ACK; validating position query as in original driver')
            pos=self.motor.query_position()
            if not math.isfinite(pos): raise RuntimeError('Invalid position')
            self.fb={'position':pos,'current_a':None,'temperature_c':None,'error':'0 (OK)'}
            self.fb_time=time.monotonic(); self.power_ready_at=time.monotonic()+(0 if self.mode=="simulation" else 5.2)
            self.emit('connection',connected=True,motor_id=self.motor.motor_id)
            self.emit('log',message='Simulation connected. Demo calibration 0–90 deg; press ARM.' if self.mode=='simulation' else 'Connected without motion. Wait 5.2 s, calibrate and Arm.')
        except Exception:
            self.can.close(); self.can=self.motor=self.fb=None; raise
    def move(self,raw,remote=False):
        if self.pending: raise RuntimeError('Busy: previous VR5 operation is pending')
        if not self.can or not self.armed: raise RuntimeError('Connect, calibrate and Arm first')
        if time.monotonic()<self.power_ready_at: raise RuntimeError('Motor startup wait not finished')
        # Read fresh position; actual current/temp validated on first type-2 response.
        pos=self.motor.query_position(timeout=.25)
        initial={'position':pos,'current_a':0.,'temperature_c':0.,'error':'0 (OK)'}
        self.seq.start(raw,self.settings,initial,time.monotonic()); self.pending=remote
        self.fb_time=time.monotonic(); self.last_cycle=time.monotonic(); self.last_send_time=time.monotonic()
        self.emit('log',message=f'MOVE {raw} → {self.seq.target:.3f} deg; current threshold {self.seq.current_limit:.1f} A')
    def receive_feedback(self,timeout=.035):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not self.stop_requested.is_set():
            fr=self.can.recv_can(timeout=min(.01,max(.001,deadline-time.monotonic())))
            if fr and fr.can_id==self.motor.motor_id:
                fb=self.motor.parse_feedback_type2(fr)
                if fb: return fb
        return None
    def handle(self,kind,args):
        if kind=='connect': self.connect(**args)
        elif kind=='disconnect': self.disconnect()
        elif kind=='settings':
            if self.seq.active: raise RuntimeError('STOP before changing calibration/settings')
            self.settings=args['settings']; self.armed=False
            if self.mode=='simulation' and self.can:
                self.can.open_deg=self.settings.open_deg; self.can.closed_deg=self.settings.closed_deg
        elif kind=='sim_object':
            if self.mode=='simulation' and self.can: self.can.object_present=args['present']
        elif kind=='arm':
            if not args['enabled']:
                self.armed=False; self.abort('Disarmed')
            else:
                if not self.can: raise RuntimeError('Not connected')
                self.settings.validate(); self.armed=True
        elif kind=='move': self.move(args['raw'])
        elif kind=='udp':
            if args['enabled'] and not self.udp:
                sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                try: sock.bind(('127.0.0.1',5050)); sock.setblocking(False)
                except Exception: sock.close(); raise
                self.udp=sock; self.emit('log',message='UDP listening :5050, feedback :5051')
            elif not args['enabled'] and self.udp:
                self.abort('UDP disabled'); self.udp.close(); self.udp=None
        else: raise ValueError('Unknown local request')
    def run(self):
        try:
            while not self.quit_requested.is_set():
                cycle=time.monotonic()
                if self.stop_requested.is_set():
                    self.armed=False; self.abort('STOP requested'); self.stop_requested.clear()
                    # Flush queued actions so an old MOVE cannot restart after STOP.
                    while True:
                        try: self.commands.get_nowait()
                        except queue.Empty: break
                for _ in range(10):
                    if self.stop_requested.is_set() or self.quit_requested.is_set(): break
                    try: kind,args=self.commands.get_nowait()
                    except queue.Empty: break
                    try: self.handle(kind,args)
                    except Exception as e: self.emit('log',message=str(e))
                if self.udp:
                    for _ in range(10):
                        if self.stop_requested.is_set(): break
                        try: packet,_=self.udp.recvfrom(256)
                        except BlockingIOError: break
                        text=packet.decode('ascii',errors='replace').strip(); self.emit('log',message='RX '+text)
                        try: self.move(parse_vr5(text),remote=True)
                        except Exception as e:
                            self.emit('log',message=str(e)); self.reply('GRIPPER_ABORTED')
                if self.seq.active and not self.stop_requested.is_set():
                    try:
                        if time.monotonic()-self.last_send_time>.25: raise RuntimeError('Servo command interval exceeded 250 ms')
                        payload=self.motor.pack_servo_position(self.seq.target,self.settings.speed_rpm,self.seq.current_limit,ack=2)
                        self.can.send_can(self.motor.motor_id,payload)
                        self.last_send_time=time.monotonic()
                        fb=self.receive_feedback()
                        now=time.monotonic()
                        if fb:
                            self.fb=fb; self.fb_time=now; self.seq.update(fb,now)
                        if now-self.fb_time>.25: raise RuntimeError('Feedback lost >250 ms')
                        outcome=self.seq.take_completion()
                        if outcome:
                            self.emit('log',message=outcome+' / '+self.seq.state)
                            if self.pending: self.reply(outcome); self.pending=False
                            if outcome=='GRIPPER_ABORTED': self.armed=False
                    except Exception as e:
                        self.seq.fault(str(e)); self.armed=False
                        if self.pending: self.reply('GRIPPER_ABORTED'); self.pending=False
                        self.emit('log',message='FAULT: '+str(e))
                elif self.can and not self.stop_requested.is_set() and cycle-self.fb_time>.3:
                    try:
                        pos=self.motor.query_position(timeout=.15)
                        if not math.isfinite(pos): raise RuntimeError('Invalid position feedback')
                        self.fb={'position':pos,'current_a':None,'temperature_c':None,'error':'0 (OK)'}; self.fb_time=time.monotonic()
                    except Exception as e:
                        self.armed=False; self.seq.state='FAULT'; self.emit('log',message=str(e)); self.disconnect()
                self.last_cycle=time.monotonic()
                self.emit('status',state=self.seq.state,armed=self.armed,connected=self.can is not None,
                    active=self.seq.active,pending=self.pending,udp=self.udp is not None,fb=self.fb,mode=self.mode,
                    feedback_age=time.monotonic()-self.fb_time,error=self.seq.error,target=self.seq.target,
                    limit=self.seq.current_limit)
                self.quit_requested.wait(max(0,.05-(time.monotonic()-cycle)))
        except Exception as e:
            self.emit('log',message='Backend stopped: '+str(e))
        finally:
            self.disconnect()
            if self.udp: self.udp.close(); self.udp=None
            self.emit('finished')
