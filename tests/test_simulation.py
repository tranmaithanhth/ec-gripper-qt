import sys,time,queue,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from backend import Backend
from control import Settings

class SimulationTests(unittest.TestCase):
    def wait(self,predicate,timeout=5):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.01)
        self.fail('Simulation timed out')
    def test_close_contact_hold_open_and_stop(self):
        b=Backend(queue.Queue());b.start()
        try:
            b.request('connect',port='',motor_id=None,mode='simulation')
            self.wait(lambda:b.fb is not None)
            b.request('settings',settings=Settings(open_deg=0,closed_deg=90,speed_rpm=5))
            b.request('arm',enabled=True);self.wait(lambda:b.armed)
            b.request('move',raw=1000);self.wait(lambda:b.seq.state=='HOLDING')
            self.assertAlmostEqual(b.fb['position'],49.5);self.assertAlmostEqual(b.seq.current_limit,.3)
            b.request('move',raw=0);self.wait(lambda:b.seq.state=='AT_POSITION')
            self.assertAlmostEqual(b.fb['position'],0,delta=.25)
            b.stop_requested.set();self.wait(lambda:not b.seq.active)
        finally:b.quit_requested.set();b.join(3)
        self.assertFalse(b.is_alive())

if __name__=='__main__':unittest.main()
