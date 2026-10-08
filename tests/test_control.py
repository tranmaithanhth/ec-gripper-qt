import sys, unittest, math, struct
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from control import Settings, Sequencer, parse_vr5
from ec_a4310_waveshare_test import EC_A4310

def fb(pos,current=0,temp=25,error='0 (OK)'):
    return dict(position=pos,current_a=current,temperature_c=temp,error=error)

class ControlTests(unittest.TestCase):
    def test_mapping_both_directions(self):
        for o,c in [(10,100),(100,10)]:
            s=Settings(open_deg=o,closed_deg=c)
            self.assertAlmostEqual(s.position(700),o+.7*(c-o))
            self.assertEqual(s.position(0),o); self.assertEqual(s.position(1000),c)
    def test_reject_bad_settings_and_packets(self):
        for s in [Settings(),Settings(open_deg=0,closed_deg=0),Settings(open_deg=0,closed_deg=90,hold_a=4),Settings(open_deg=0,closed_deg=90,speed_rpm=float('nan'))]:
            with self.assertRaises(ValueError): s.validate()
        for packet in ['GRIPPER_CLOSE:-1','GRIPPER_CLOSE:1001','GRIPPER_CLOSE:nan','GRIPPER_CLOSE:7.0','GRIPPER_CLOSE:٧','STOP']:
            with self.assertRaises(ValueError): parse_vr5(packet)
        self.assertEqual(parse_vr5('GRIPPER_CLOSE:700'),700)
        self.assertEqual(parse_vr5('GRIPPER_OPEN'),0)
    def test_contact_then_reduce_current_then_complete(self):
        q=Sequencer(); s=Settings(open_deg=0,closed_deg=90)
        q.start(700,s,fb(0),0)
        for i in range(1,31):
            t=i*.05; q.update(fb(min(30,t*100),.45 if t>=.3 else .1),t)
        self.assertEqual(q.state,'HOLDING'); self.assertEqual(q.current_limit,.3)
        self.assertAlmostEqual(q.target,31); self.assertEqual(q.take_completion(),'GRIPPER_COMPLETE')
        self.assertTrue(q.active); self.assertIsNone(q.take_completion())
    def test_open_high_current_is_not_contact(self):
        q=Sequencer(); s=Settings(open_deg=0,closed_deg=90)
        q.start(0,s,fb(50),0)
        for i in range(1,20): q.update(fb(40,.5),i*.05)
        self.assertEqual(q.state,'MOVING')
    def test_no_object_reaches_target(self):
        q=Sequencer(); s=Settings(open_deg=0,closed_deg=90)
        q.start(700,s,fb(0),0)
        for i in range(1,20): q.update(fb(63),i*.05)
        self.assertEqual(q.state,'AT_POSITION'); self.assertEqual(q.take_completion(),'GRIPPER_COMPLETE')
    def test_fault_temperature_nan_error_travel_current_timeout(self):
        for feedback in [fb(20,temp=75),fb(float('nan')),fb(20,error='2 (overcurrent)'),fb(100),fb(20,current=2)]:
            q=Sequencer(); q.start(700,Settings(open_deg=0,closed_deg=90),fb(0),0); q.update(feedback,.05)
            self.assertFalse(q.active); self.assertEqual(q.take_completion(),'GRIPPER_ABORTED')
        q=Sequencer();q.start(700,Settings(open_deg=0,closed_deg=90),fb(0),0);q.update(fb(0),21)
        self.assertEqual(q.state,'FAULT')
    def test_native_can_payload_not_new_protocol(self):
        p=EC_A4310.pack_servo_position(63,1,.5,ack=2); raw=int.from_bytes(p,'big')
        self.assertEqual(raw>>61,1); self.assertEqual(raw&3,2)
        self.assertEqual((raw>>2)&0xfff,5); self.assertEqual((raw>>14)&0x7fff,10)
        pos=struct.unpack('>f',((raw>>29)&0xffffffff).to_bytes(4,'big'))[0]
        self.assertAlmostEqual(pos,63)

if __name__=='__main__': unittest.main()
