"""Stationary, exact-time grid/TF snapshot. Never extrapolate or widen freshness."""
import math
import time


def grid_context(node, event=None, timeout=1.5):
    from tf2_ros import TransformException
    guard = node.recovery_guard
    if node.active_goal_handle is not None:
        raise RuntimeError("Stationary TF synchronization requested during navigation")
    started = time.monotonic()
    deadline = started + timeout
    msg = None
    last = "no grid received"
    attempts = 0
    while time.monotonic() < deadline:
        try:
            if msg is None:
                msg = guard.fresh('grid', .75)
                receipt = guard.data['grid'][1]
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp_s
            if not -.1 <= age <= .75 or time.monotonic() - receipt > .75:
                msg = None
                last = "pinned grid expired while waiting for TF"
            else:
                attempts += 1
                # Pin this timestamp while callbacks catch up; do not chase newer grids.
                robot = guard.transform(msg.header.frame_id, 'base_footprint', stamp=msg.header.stamp)
                map_to_grid = guard.transform(msg.header.frame_id, 'map', stamp=msg.header.stamp)
                odom = guard.fresh('odom', .5).twist.twist
                speeds = (odom.linear.x, odom.linear.y, odom.angular.z)
                if (not all(math.isfinite(v) for v in speeds)
                        or math.hypot(*speeds[:2]) >= .02 or abs(speeds[2]) >= .03):
                    raise ValueError("Robot moved during stationary TF synchronization")
                age = node.get_clock().now().nanoseconds / 1e9 - stamp_s
                if -.1 <= age <= .75 and time.monotonic() - receipt <= .75:
                    if event and attempts > 1:
                        event('tf_synchronized', attempts=attempts,
                              waited_s=round(time.monotonic() - started, 3), grid_stamp=stamp_s)
                    return msg, robot, map_to_grid
                msg = None
        except TransformException as error:
            last = str(error)
        except RuntimeError as error:
            # Freshness failures are retried while stationary, within the same deadline.
            last = str(error)
            msg = None
        guard.spin(.02)
    raise RuntimeError("No fresh synchronized grid/TF within %.1fs: %s" % (timeout, last))
