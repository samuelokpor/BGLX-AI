import unittest
from types import SimpleNamespace
from wiring import attach_advisory,execute_leg

class IntegrationTests(unittest.TestCase):
    def node(self,backup=True):
        calls=[]
        n=SimpleNamespace(active_goal_handle=None)
        n.recovery_guard=SimpleNamespace(ready=lambda:calls.append('ready'),backup=lambda reason:(calls.append('backup'),backup)[1])
        n.passage_alignment=SimpleNamespace(perform=lambda candidate,deadline:(calls.append(('original_alignment',candidate,deadline)),True)[1])
        n.navigate_with_exploration=lambda name,target:(calls.append(('existing_navigation',name,target)),True)[1]
        return n,calls
    def test_backup_failure_blocks_home(self):
        n,c=self.node(False)
        with self.assertRaises(RuntimeError):execute_leg(n,'return-home',True)
        self.assertEqual(c,['backup'])
    def test_backup_then_original_navigation(self):
        n,c=self.node();self.assertTrue(execute_leg(n,'return-home',True))
        self.assertEqual(c,['backup',('existing_navigation','HOME',(0.,0.,0.))])
    def test_c_uses_original_navigation(self):
        n,c=self.node();execute_leg(n,'home-to-c')
        self.assertEqual(c[0][0:2],('existing_navigation','DELIVERY_C'))
    def test_advisory_then_same_alignment(self):
        n,c=self.node();candidate=object()
        attach_advisory(n,lambda gate,deadline:c.append(('vision',gate,deadline)))
        self.assertTrue(n.passage_alignment.perform(candidate,123))
        self.assertEqual(c,['ready',('vision',candidate,123),'ready',('original_alignment',candidate,123)])
    def test_no_vision_during_navigation(self):
        n,c=self.node();n.active_goal_handle=object();attach_advisory(n,lambda *_:c.append('vision'))
        with self.assertRaises(RuntimeError):n.passage_alignment.perform(None,123)
        self.assertEqual(c,[])
    def test_capture_failure_blocks_alignment(self):
        n,c=self.node()
        def fail(*_):raise RuntimeError('camera capture failed')
        attach_advisory(n,fail)
        with self.assertRaises(RuntimeError):n.passage_alignment.perform(None,123)
        self.assertEqual(c,['ready'])

if __name__=='__main__':unittest.main()
