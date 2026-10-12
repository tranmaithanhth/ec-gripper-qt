import math
import unittest
from mechanics import Geometry, read_config

class GeometryTests(unittest.TestCase):
    def setUp(self):
        # Mathematical fixture, not a hardware preset.
        self.inputs=Geometry(max_mm=80,r_mm=20,l_mm=100,e_mm=0,doffset_mm=-160)
        self.g=self.inputs.solve()[0]
    def test_auto_angles_and_nonlinear_width(self):
        self.assertAlmostEqual(self.g.crank_open_deg,0)
        self.assertAlmostEqual(self.g.crank_closed_deg,180)
        self.assertAlmostEqual(self.g.at_fraction(0),80)
        self.assertAlmostEqual(self.g.at_fraction(1),0)
        self.assertAlmostEqual(self.g.at_fraction(.5),2*math.sqrt(9600)-160)
        self.assertNotAlmostEqual(self.g.at_fraction(.5),40)
    def test_independent_inputs_not_rewritten(self):
        values=self.inputs.__dict__.copy()
        self.inputs.solve()
        self.assertEqual(self.inputs.__dict__,values)
        g=Geometry(max_mm=60,r_mm=20,l_mm=100,doffset_mm=-160).solve()[0]
        self.assertAlmostEqual(g.at_fraction(0),60)
        self.assertAlmostEqual(g.at_fraction(1),0)
    def test_offset_and_multiple_branches(self):
        base=Geometry(r_mm=20,l_mm=100,e_mm=10)
        offset=-2*base.x(140)
        inputs=Geometry(max_mm=offset+2*base.x(20),r_mm=20,l_mm=100,e_mm=10,doffset_mm=offset)
        branches=inputs.solve();self.assertGreaterEqual(len(branches),2)
        self.assertTrue(any(abs(g.crank_open_deg-20)<1e-6 and abs(g.crank_closed_deg-140)<1e-6 for g in branches))
        for g in branches:
            self.assertAlmostEqual(g.at_fraction(0),inputs.max_mm)
            self.assertAlmostEqual(g.at_fraction(1),0)
    def test_encoder_mapping_both_directions(self):
        for a,b in [(9,94),(94,9)]:
            self.assertAlmostEqual(self.g.width(a,a,b),80)
            self.assertAlmostEqual(self.g.width(b,a,b),0)
            self.assertAlmostEqual(self.g.width((a+b)/2,a,b),self.g.at_fraction(.5))
    def test_missing_unreachable_and_nonfinite(self):
        for g in [Geometry(),Geometry(max_mm=90,r_mm=20,l_mm=100,doffset_mm=-160),Geometry(r_mm=20,l_mm=100,doffset_mm=0),Geometry(r_mm=float('nan'))]:
            with self.assertRaises(ValueError): g.solve()
    def test_no_silent_clipping(self):
        for args in [(100,0,90),(0,0,0),(float('nan'),0,90)]:
            with self.assertRaises(ValueError): self.g.width(*args)
    def test_positive_root_branch_filter(self):
        g=Geometry(r_mm=100,l_mm=20)
        for t in g.roots(160): self.assertAlmostEqual(2*g.x(t),160)
    def test_config_migration_and_draft(self):
        inputs,preferred=read_config(dict(max_mm=80,closed_mm=0,r_mm=20,l_mm=100,e_mm=0,crank_open_deg=0,crank_closed_deg=180))
        self.assertEqual(inputs,self.inputs);self.assertEqual(preferred,[0,180])
        draft,preferred=read_config(dict(max_mm=80,r_mm=0,l_mm=0,e_mm=0,doffset_mm=0,version=2,selected_angles=None))
        self.assertEqual(draft.r_mm,0)
        with self.assertRaises(ValueError):read_config(dict(max_mm=80,closed_mm=5,r_mm=20,l_mm=100,e_mm=0,crank_open_deg=0,crank_closed_deg=180))

if __name__=='__main__': unittest.main()
