import sys, time, queue, socket, unittest
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from backend import Backend
from control import Settings
from ec_a4310_waveshare_test import EC_A4310

class FakeCan:
    missing=False
    def __init__(self,*a,**kw): self.position=0; self.target=0; self.closed=False; self.sent=[]
    def configure_1m_standard_normal(self): pass
    def send_can(self,mid,payload): self.sent.append(payload); self.position=self.target
    def recv_can(self,timeout):
        time.sleep(.001)
        return None if self.missing else object()
    def close(self): self.closed=True

class FakeMotor:
    pack_servo_position=staticmethod(EC_A4310.pack_servo_position)
    def __init__(self,can,motor_id): self.can=can; self.motor_id=motor_id
    def discover_id(self): self.motor_id=1; return 1
    def set_qna_mode(self): return True
    def query_position(self,timeout=.5): return self.can.position
    def parse_feedback_type2(self,fr): return dict(position=self.can.position,current_a=.1,temperature_c=25,error='0 (OK)')

class Frame:
    can_id=1

class BackendTests(unittest.TestCase):
    def wait_until(self,fn,timeout=2):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if fn(): return
            time.sleep(.01)
        self.fail('Timed out')
    def test_disconnect_releases_udp_port(self):
        b=Backend(queue.Queue())
        b.handle('udp',dict(enabled=True))
        sock=b.udp
        try:
            b.disconnect()
            self.assertIsNone(b.udp)
            self.assertFalse(b.armed)
            self.assertEqual(sock.fileno(),-1)
            # A fresh session can bind immediately after Disconnect.
            b.handle('udp',dict(enabled=True))
            self.assertIsNotNone(b.udp)
        finally:
            b.disconnect()

    def test_udp_completion_heartbeat_and_stop(self):
        with patch('ec_a4310_waveshare_test.WaveshareUsbCanA',FakeCan),patch('ec_a4310_waveshare_test.EC_A4310',FakeMotor),patch.object(FakeCan,'recv_can',lambda self,timeout: None if self.missing else Frame()):
            b=Backend(queue.Queue()); rx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); tx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            rx.bind(('127.0.0.1',5051)); rx.settimeout(3); b.start()
            try:
                b.request('connect',port='FAKE',motor_id=1);self.wait_until(lambda:b.motor is not None)
                b.power_ready_at=0; b.request('settings',settings=Settings(open_deg=0,closed_deg=90)); b.request('arm',enabled=True); b.request('udp',enabled=True)
                self.wait_until(lambda:b.udp is not None and b.armed)
                b.can.target=63;tx.sendto(b'GRIPPER_CLOSE:700',('127.0.0.1',5050))
                self.assertEqual(rx.recvfrom(256)[0],b'GRIPPER_COMPLETE'); self.assertTrue(b.seq.active)
                self.assertAlmostEqual(b.seq.target,63); before=len(b.can.sent);time.sleep(.15);self.assertGreater(len(b.can.sent),before)
                b.stop_requested.set();self.wait_until(lambda:not b.seq.active);self.assertFalse(b.armed)
                count=len(b.can.sent);time.sleep(.15);self.assertEqual(len(b.can.sent),count)
                tx.sendto(b'GRIPPER_OPEN',('127.0.0.1',5050));self.assertEqual(rx.recvfrom(256)[0],b'GRIPPER_ABORTED')
                # Missing feedback during an accepted command causes ABORT and stops heartbeat.
                b.request('arm',enabled=True);self.wait_until(lambda:b.armed)
                b.can.missing=True;tx.sendto(b'GRIPPER_CLOSE:1000',('127.0.0.1',5050))
                self.assertEqual(rx.recvfrom(256)[0],b'GRIPPER_ABORTED');self.assertFalse(b.seq.active)
            finally:
                b.quit_requested.set();b.join(3);rx.close();tx.close()
            self.assertFalse(b.is_alive())

if __name__=='__main__': unittest.main()
