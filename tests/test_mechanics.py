import math
import unittest
from mechanics import Geometry

class GeometryTests(unittest.TestCase):
    def setUp(self):
        # Analytic test fixture only, NOT the user's optimized dimensions.
        self.g=Geometry(r_mm=20,l_mm=100,crank_open_deg=0,crank_closed_deg=180).validate()
    def test_endpoints_and_nonlinear_midpoint(self):
        self.assertAlmostEqual(self.g.at_fraction(0),80)
        self.assertAlmostEqual(self.g.at_fraction(1),0)
        self.assertAlmostEqual(self.g.at_fraction(.5),2*(math.sqrt(9600)-80))
        self.assertNotAlmostEqual(self.g.at_fraction(.5),40)
    def test_encoder_mapping_both_directions(self):
        for a,b in [(9,94),(94,9)]:
            self.assertAlmostEqual(self.g.width(a,a,b),80)
            self.assertAlmostEqual(self.g.width(b,a,b),0)
            self.assertAlmostEqual(self.g.width((a+b)/2,a,b),self.g.at_fraction(.5))
    def test_unconfigured_mismatch_and_invalid_geometry(self):
        for g in [Geometry(),Geometry(max_mm=70,r_mm=20,l_mm=100,crank_closed_deg=180),Geometry(r_mm=50,l_mm=20,crank_closed_deg=180),Geometry(r_mm=20,l_mm=100,crank_open_deg=180,crank_closed_deg=0),Geometry(r_mm=float('nan'))]:
            with self.assertRaises(ValueError): g.validate()
    def test_no_silent_clipping_or_bad_calibration(self):
        for args in [(100,0,90),(0,0,0),(float('nan'),0,90)]:
            with self.assertRaises(ValueError): self.g.width(*args)

if __name__=='__main__': unittest.main()
