
import ast
import tempfile
import unittest
from pathlib import Path
from protocol import decision, rank_candidates

class ProtocolTests(unittest.TestCase):
    def result(self, stage='forward', code='SUCCESS', **kw):
        return dict(schema=1, stage=stage, code=code, stopped=True, cleanup_ok=True, **kw)
    def test_success(self):
        for stage in ('scene','forward','reverse'):
            self.assertEqual(decision(self.result(stage),stage,0),'NEXT')
    def test_recovery_only_known_failures(self):
        for code in ('CLEARANCE','TRACKING','NO_ROUTE'):
            self.assertEqual(decision(self.result(code=code),'forward',10),'RECOVER')
        for code in ('TF_STALE','ERROR','INTERRUPTED','CLEANUP_FAILED'):
            self.assertEqual(decision(self.result(code=code),'forward',10),'STOP')
    def test_bounded(self):
        self.assertEqual(decision(self.result(code='CLEARANCE'),'forward',10,True),'STOP')
    def test_clean_stop_required(self):
        for field in ('stopped','cleanup_ok'):
            data=self.result(code='CLEARANCE');data[field]=False
            self.assertEqual(decision(data,'forward',10),'STOP')
    def test_endpoint(self):
        for distance, expected in ((.03,'NEXT'),(.09,'STOP'),(float('nan'),'STOP'),(True,'STOP')):
            data=self.result('reverse','REVERSE_ENDPOINT',metrics={'endpoint_distance':distance})
            self.assertEqual(decision(data,'reverse',10),expected)
    def test_missing_and_exit(self):
        self.assertEqual(decision({},'forward',0),'STOP')
        self.assertEqual(decision(self.result(code='CLEARANCE'),'forward',0),'STOP')
        self.assertEqual(decision(self.result(),'reverse',0),'STOP')
    def test_earlier_alignment(self):
        a=(.18,-7,2.2);b=(.178,-7.2,3.5);c=(.16,-8,4)
        self.assertEqual(rank_candidates([a,b,c]),[b,a,c])
    def test_stage_instrumentation(self):
        root=Path(__file__).parent
        for name in ('forward.py','reverse.py','scene.py'):
            tree=ast.parse((root/name).read_text())
            self.assertIsInstance(tree.body[-1],ast.Raise)
            self.assertIn('report.finish()',ast.unparse(tree))
            self.assertIn("report.data['code'] = 'SUCCESS'",ast.unparse(tree))
        tree=ast.parse((root/'forward.py').read_text())
        codes=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant)]
        self.assertIn('CLEARANCE',codes);self.assertIn('TRACKING',codes)
        self.assertIn('NO_ROUTE',codes)

if __name__=='__main__':unittest.main()
