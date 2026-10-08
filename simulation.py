"""Synthetic position/current feedback; no hardware or Simulink required."""
import struct, time
from ec_a4310_waveshare_test import EC_A4310

class SimCan:
    def __init__(self):
        self.position=0.; self.target=0.; self.speed=1.; self.limit=.5
        self.current=0.; self.last=time.monotonic(); self.object_present=True
        self.open_deg=0.; self.closed_deg=90.; self.closed=False
    def configure_1m_standard_normal(self): pass
    def close(self): self.closed=True
    def integrate(self):
        now=time.monotonic();dt=min(.1,now-self.last);self.last=now
        delta=self.target-self.position; step=min(abs(delta),self.speed*6*dt)
        proposed=self.position+(step if delta>=0 else -step)
        sign=1 if self.closed_deg>self.open_deg else -1
        barrier=self.open_deg+.55*(self.closed_deg-self.open_deg)
        contact=self.object_present and delta*sign>0 and (proposed-barrier)*sign>=0
        if contact:
            self.position=barrier;self.current=self.limit*sign
        else:
            self.position=proposed;self.current=(min(.1,self.limit)*(1 if delta>0 else -1)) if abs(delta)>.01 else 0.
    def send_can(self,mid,payload):
        self.integrate();raw=int.from_bytes(payload,'big')
        self.target=struct.unpack('>f',((raw>>29)&0xffffffff).to_bytes(4,'big'))[0]
        self.speed=((raw>>14)&0x7fff)/10;self.limit=((raw>>2)&0xfff)/10
    def recv_can(self,timeout):
        self.integrate()
        return type('Frame',(),{'can_id':1})()

class SimMotor:
    pack_servo_position=staticmethod(EC_A4310.pack_servo_position)
    def __init__(self,can,motor_id=1): self.can=can;self.motor_id=1
    def discover_id(self):return 1
    def set_qna_mode(self):return True
    def query_position(self,timeout=.5):return self.can.position
    def parse_feedback_type2(self,frame):
        return dict(position=self.can.position,current_a=self.can.current,temperature_c=25.,error='0 (SIMULATED)')
