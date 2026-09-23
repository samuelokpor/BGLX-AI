"""No ROS required: timing, candidate policy, patch preservation and ownership."""
import ast
from pathlib import Path
import math
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import bglx_tf_sync
from install import patch_handoff
from live_policy import evaluate
from run import InspectOpening, LiveVision, transit_then_inspect


class PolicyTests(unittest.TestCase):
    def scene(self, offset=(0., 0.)):
        dx, dy = offset
        targets = [dict(mouth=[4. + dx, y + dy, 1.192], normal=[1., 0.], width=1.8)
                   for y in (25.5, 27.8, 30.05, 34.55)]
        return targets, (dx, 20. + dy, math.pi / 2), (28. + dx, 82. + dy, 0.)

    def test_first_nearby_opening_not_previous_waypoint(self):
        eligible, rows = evaluate(*self.scene())
        self.assertEqual(len(eligible), 1)
        self.assertEqual(eligible[0]['target']['mouth'][1], 25.5)
        self.assertEqual(rows[-1]['reason'], 'outside inspection distance')

    def test_no_site_coordinate_dependency(self):
        a, _ = evaluate(*self.scene())
        b, _ = evaluate(*self.scene((101., -63.)))
        self.assertAlmostEqual(a[0]['distance_m'], b[0]['distance_m'])
        self.assertAlmostEqual(b[0]['target']['mouth'][0] - a[0]['target']['mouth'][0], 101.)
        self.assertAlmostEqual(b[0]['target']['mouth'][1] - a[0]['target']['mouth'][1], -63.)

    def test_geometry_rejection(self):
        targets, robot, goal = self.scene()
        targets[0]['width'] = .7
        eligible, _ = evaluate(targets, robot, goal)
        self.assertFalse(eligible)
        targets[0]['width'] = 1.8
        targets[0]['normal'] = [-1., 0.]
        eligible, _ = evaluate(targets, robot, goal)
        self.assertFalse(eligible)


class TFProblem(Exception):
    pass


class TFTests(unittest.TestCase):
    def node(self, permanent=False, moving=False):
        wall = [0.]
        stamp = NS(sec=100, nanosec=10000000)
        msg = NS(header=NS(stamp=stamp, frame_id='odom'))
        requested = []
        def fresh(key, limit):
            if key == 'grid':
                if wall[0] > .75:
                    raise RuntimeError('missing/stale grid')
                return msg
            return NS(twist=NS(twist=NS(linear=NS(x=.1 if moving else 0., y=0.), angular=NS(z=0.))))
        def transform(target, source, stamp):
            requested.append((target, source, stamp.sec, stamp.nanosec))
            if permanent or wall[0] < .017:
                raise TFProblem('Requested 100.010; latest 99.993')
            return 0., 0., 0.
        guard = NS(fresh=fresh, transform=transform, data={'grid': (msg, 0.)},
                   spin=lambda dt: wall.__setitem__(0, wall[0] + dt))
        clock = NS(now=lambda: NS(nanoseconds=int((100.02 + wall[0]) * 1e9)))
        node = NS(active_goal_handle=None, recovery_guard=guard, get_clock=lambda: clock)
        return node, wall, requested

    def test_waits_for_17ms_lag_at_same_stamp(self):
        node, wall, requested = self.node()
        with patch.dict(sys.modules, {'tf2_ros': NS(TransformException=TFProblem)}), \
                patch.object(bglx_tf_sync.time, 'monotonic', side_effect=lambda: wall[0]):
            msg, robot, mapping = bglx_tf_sync.grid_context(node)
        self.assertGreaterEqual(wall[0], .017)
        self.assertLess(wall[0], .1)
        self.assertEqual({r[2:] for r in requested}, {(100, 10000000)})

    def test_missing_tf_still_fails_bounded(self):
        node, wall, _ = self.node(permanent=True)
        with patch.dict(sys.modules, {'tf2_ros': NS(TransformException=TFProblem)}), \
                patch.object(bglx_tf_sync.time, 'monotonic', side_effect=lambda: wall[0]):
            with self.assertRaisesRegex(RuntimeError, 'No fresh synchronized'):
                bglx_tf_sync.grid_context(node)
        self.assertLess(wall[0], 1.53)

    def test_robot_motion_is_not_accepted(self):
        node, wall, _ = self.node(moving=True)
        with patch.dict(sys.modules, {'tf2_ros': NS(TransformException=TFProblem)}), \
                patch.object(bglx_tf_sync.time, 'monotonic', side_effect=lambda: wall[0]):
            with self.assertRaisesRegex(ValueError, 'Robot moved'):
                bglx_tf_sync.grid_context(node)

    def test_active_navigation_cannot_wait(self):
        node, _, _ = self.node()
        node.active_goal_handle = object()
        with patch.dict(sys.modules, {'tf2_ros': NS(TransformException=TFProblem)}):
            with self.assertRaisesRegex(RuntimeError, 'during navigation'):
                bglx_tf_sync.grid_context(node)


class HandoffTests(unittest.TestCase):
    def fixtures(self, cancel=True, triggers=True):
        sequence = []
        def navigate(*args):
            sequence.append('navigate')
            if triggers:
                raise InspectOpening()
            return True
        def stop():
            sequence.append('cancel_and_confirm_stop')
            return cancel
        def inspect(execute):
            sequence.append('inspect')
            return True
        node = NS(navigate_with_exploration=navigate, recovery_guard=NS(cancel_navigation=stop))
        bridge = NS(destination=(28., 82., 0.), event=lambda *a, **k: None, inspect=inspect)
        return node, bridge, sequence

    def test_cancel_before_inspection(self):
        node, bridge, sequence = self.fixtures()
        self.assertTrue(transit_then_inspect(node, bridge))
        self.assertEqual(sequence, ['navigate', 'cancel_and_confirm_stop', 'inspect'])

    def test_cancel_failure_blocks_inspection(self):
        node, bridge, sequence = self.fixtures(cancel=False)
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            transit_then_inspect(node, bridge)
        self.assertNotIn('inspect', sequence)

    def test_arrival_without_vision_is_not_trial_success(self):
        node, bridge, _ = self.fixtures(triggers=False)
        self.assertFalse(transit_then_inspect(node, bridge))

    def test_original_poll_is_preserved_after_crossing(self):
        bridge = object.__new__(LiveVision)
        bridge.armed = False
        bridge.original_poll = lambda: 'existing_alignment_candidate'
        self.assertEqual(bridge.poll(), 'existing_alignment_candidate')


class PatchTests(unittest.TestCase):
    SOURCE = '''"""Preserve core behaviour."""
def run(node,event):
 guard=node.recovery_guard
 values=[]
 if not 0<values[1].double_value<=1.5:raise RuntimeError('speed')
 for cid in []:
  msg=guard.fresh('grid',.75);frame=msg.header.frame_id;grid=Grid(msg)
  robot=guard.transform(frame,'base_footprint',stamp=msg.header.stamp)
  mouth=xf((1,2,3),guard.transform(frame,'map',stamp=msg.header.stamp))
  if not grid.path_clear([robot,mouth]):raise RuntimeError('geometry')
 return finish_after_crossing(node)
'''

    def test_patch_retains_gates_and_avoids_duplicate_lookups(self):
        fixed = patch_handoff(self.SOURCE)
        self.assertEqual(patch_handoff(fixed), fixed)
        self.assertIn('<= 1.5', fixed)
        self.assertIn('grid.path_clear', fixed)
        self.assertIn('finish_after_crossing(node)', fixed)
        self.assertNotIn('guard.transform', fixed)
        self.assertIn('bglx_grid_context(node, event=event)', fixed)
        compile(fixed, 'patched.py', 'exec')

    def test_unexpected_layout_refused(self):
        with self.assertRaises(RuntimeError):
            patch_handoff(self.SOURCE.replace('robot=guard.transform', 'other=guard.transform'))


if __name__ == '__main__':
    unittest.main()
