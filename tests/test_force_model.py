import math
import unittest
from mechanics import Geometry
from force_model import ForceCalibration, estimate_forces

class ForceTests(unittest.TestCase):
    def setUp(self):
        self.g=Geometry(80,20,100,0,-160).solve()[0]
        self.c=ForceCalibration(2,.7,90)
    def test_virtual_work_and_v_contacts(self):
        # Full-gap derivative = -0.04 m/rad at 90 degrees.
        flat,v=estimate_forces(self.g,90,0,180,1,self.c)
        self.assertAlmostEqual(flat,70)
        self.assertAlmostEqual(v,70*math.sqrt(2))
        self.assertEqual(estimate_forces(self.g,90,0,180,0,self.c),(0,0))
        self.assertEqual(estimate_forces(self.g,90,0,180,-1,self.c),(flat,v))
    def test_encoder_ratio_and_direction(self):
        self.assertAlmostEqual(estimate_forces(self.g,45,0,90,1,self.c)[0],35)
        self.assertAlmostEqual(estimate_forces(self.g,45,90,0,1,self.c)[0],35)
    def test_finite_difference_nonzero_e(self):
        base=Geometry(r_mm=20,l_mm=100,e_mm=10)
        d=-2*base.x(140)
        g=Geometry(d+2*base.x(20),20,100,10,d).solve()[0]
        h=1e-4; q=40
        derivative=(g.width(q+h,0,100)-g.width(q-h,0,100))/(2*math.radians(h))/1000
        force=estimate_forces(g,q,0,100,1,self.c)[0]
        self.assertAlmostEqual(force,2*.7*2/abs(derivative),places=5)
    def test_invalid_and_singular(self):
        for position,current,cal in [(0,1,self.c),(180,1,self.c),(181,1,self.c),(90,None,self.c),(90,float('nan'),self.c),(90,1,ForceCalibration()),(90,1,ForceCalibration(2,.7,0)),(90,1,ForceCalibration(2,2,90))]:
            with self.assertRaises(ValueError): estimate_forces(self.g,position,0,180,current,cal)
    def test_flat_limit(self):
        flat,v=estimate_forces(self.g,90,0,180,1,ForceCalibration(2,.7,180))
        self.assertAlmostEqual(flat,v)

if __name__=='__main__': unittest.main()
