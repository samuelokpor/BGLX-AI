"""ROS-independent tests of the actual runner's geometric screening."""
import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from geometry import xf, poly_dist

source=ast.parse(Path(__file__).with_name('runner.py').read_text())
wanted={'yaw','wrap','clearance','audit','reverse_poses','heading_error_at'}
ns={'math':math,'geo':{'xf':xf,'poly_dist':poly_dist},'PLAN_CLEARANCE':.17,
    'FP':[(-.38,-.52),(1.63,-.52),(1.63,.52),(-.38,.52)],
    'cx':4.,'cy':0.,'nx':1.,'ny':0.,'GAP':1.4}
exec(compile(ast.Module(body=[n for n in source.body if isinstance(n,ast.FunctionDef) and n.name in wanted],type_ignores=[]),'runner.py','exec'),ns)
square=[(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)]
ns['boxes']=[xf(square,4.,v,0.) for v in (1.,-1.)]
def path(points):
 return S(poses=[S(pose=S(position=S(x=x,y=y),orientation=S(x=0.,y=0.,z=math.sin(a/2),w=math.cos(a/2)))) for x,y,a in points])
class Screening(unittest.TestCase):
 def test_centered_passage(self):
  _,minimum,_=ns['audit'](path([(0,0,0),(8,0,0)]),(8,0,0),True)
  self.assertAlmostEqual(minimum,.18)
 def test_body_collision_despite_base_crossing(self):
  with self.assertRaises(RuntimeError):ns['audit'](path([(0,.3,0),(8,.3,0)]),(8,.3,0),True)
 def test_clear_but_insufficient_margin(self):
  with self.assertRaises(RuntimeError):ns['audit'](path([(0,.02,0),(8,.02,0)]),(8,.02,0),True)
 def test_around_rejected(self):
  with self.assertRaises(RuntimeError):ns['audit'](path([(0,3,0),(8,3,0)]),(8,3,0),True)
 def test_reverse_rejected_in_forward_stage(self):
  with self.assertRaises(RuntimeError):ns['audit'](path([(8,0,0),(0,0,0)]),(0,0,0),True)
 def test_retreat_rotates_with_robot(self):
  p=ns['reverse_poses']((1,2,math.pi/2),.7)[-1]
  self.assertAlmostEqual(p[0],1);self.assertAlmostEqual(p[1],1.3)
 def test_intermediate_heading_not_artificial_gate(self):
  ns['audit'](path([(0,0,0),(1,0,.03)]),(1,0,0),False)
if __name__=='__main__':unittest.main()
