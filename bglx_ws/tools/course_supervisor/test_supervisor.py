import ast
import unittest
from pathlib import Path
from run import decision
class Tests(unittest.TestCase):
 def result(self,code='SUCCESS',stage='course'):
  return dict(schema=1,run_id='token',stage=stage,code=code,stopped=True,cleanup_ok=True)
 def test_success(self):self.assertEqual(decision(self.result(),0,'token','course',True),'PASS')
 def test_audit(self):self.assertEqual(decision(self.result('AUDIT_PASS'),0,'token','course',False),'PASS')
 def test_audit_cannot_report_motion_success(self):self.assertEqual(decision(self.result(),0,'token','course',False),'STOP')
 def test_frame_retry(self):self.assertEqual(decision(self.result('FRAME_CHANGED'),10,'token','course',True),'REPLAN')
 def test_setup_no_retry(self):self.assertEqual(decision(self.result('FRAME_CHANGED','setup'),10,'token','setup',True),'STOP')
 def test_guards_stop(self):
  for code in ['CLEARANCE','TRACKING','ERROR','TF_STALE','CLEANUP_FAILED']:
   self.assertEqual(decision(self.result(code),10,'token','course',True),'STOP')
 def test_invalid_results(self):
  for key,value in [('cleanup_ok',False),('stopped',False),('run_id','wrong'),('stage','wrong')]:
   d=self.result('FRAME_CHANGED');d[key]=value
   self.assertEqual(decision(d,10,'token','course',True),'STOP')
 def test_execution_is_gated(self):
  t=ast.parse((Path(__file__).parent/'worker.py').read_text())
  m=next(n for n in t.body if isinstance(n,ast.Try))
  gate=next(n for n in m.body if isinstance(n,ast.If) and 'COURSE_EXECUTE' in ast.unparse(n.test))
  for node in m.body:
   if node is gate:continue
   calls=[n.func.id for n in ast.walk(node) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
   self.assertNotIn('execute',calls);self.assertNotIn('update_parameters',calls)
  self.assertIn('execute(',ast.unparse(gate.body));self.assertNotIn('execute(',ast.unparse(gate.orelse))
 def test_setup_does_not_send_goal(self):
  t=ast.parse((Path(__file__).parent/'setup.py').read_text())
  m=next(n for n in t.body if isinstance(n,ast.Try))
  self.assertNotIn('send_goal_async',ast.unparse(m))
if __name__=='__main__':unittest.main()
