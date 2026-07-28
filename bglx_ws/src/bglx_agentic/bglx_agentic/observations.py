"""Turn raw robot state into text a language model can reason about.

This module is deliberately free of ROS imports. It accepts any object that
looks like a LaserScan, which means it can be tested at a plain Python prompt
with no simulator running.
"""

import math

# --- Platform geometry -----------------------------------------------------
# These MUST match the cmd_vel_limiter parameters in
# bglx_navigation/launch/navigation.launch.py. If you retune the trike there,
# change them here too or every diagnosis below becomes a lie.
WHEELBASE = 1.33          # m
MAX_STEERING = 0.6        # rad
MAX_LINEAR_VEL = 2.78     # m/s

# A tricycle's tightest circle. R = L / tan(delta_max)
MIN_TURN_RADIUS = WHEELBASE / math.tan(MAX_STEERING)   # ~1.94 m

# Below this speed, steering does essentially nothing to the heading.
MIN_SPEED_FOR_YAW = 0.15  # m/s


def max_yaw_rate(speed):
    """Achievable |omega| at a given forward speed.

    Ackermann kinematics: omega = v * tan(delta) / L. Note this is ZERO at
    standstill -- the single most important fact about this platform.
    """
    return abs(speed) * math.tan(MAX_STEERING) / WHEELBASE


# --- Scan summarising ------------------------------------------------------
# Eight sectors, named from the robot's point of view. 0 deg is straight
# ahead, positive angles to the left (ROS REP-103 convention).
SECTORS = [
    ("front",       -22.5,   22.5),
    ("front-left",   22.5,   67.5),
    ("left",         67.5,  112.5),
    ("rear-left",   112.5,  157.5),
    ("rear",        157.5,  180.001),
    ("rear",       -180.0, -157.5),
    ("rear-right", -157.5, -112.5),
    ("right",      -112.5,  -67.5),
    ("front-right", -67.5,  -22.5),
]


def summarise_scan(scan, max_report=10.0):
    """LaserScan-like object -> {sector name: nearest range in metres}.

    Returns None if the scan is unusable. That is itself a reportable fact,
    not an error to swallow.
    """
    if scan is None or not getattr(scan, "ranges", None):
        return None

    buckets = {}
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r):
            continue                      # inf/NaN = no return in that ray
        if r < scan.range_min or r > scan.range_max:
            continue
        angle = math.degrees(scan.angle_min + i * scan.angle_increment)
        angle = (angle + 180.0) % 360.0 - 180.0     # wrap to [-180, 180)
        for name, lo, hi in SECTORS:
            if lo <= angle < hi:
                cur = buckets.get(name)
                if cur is None or r < cur:
                    buckets[name] = r
                break

    if not buckets:
        return None
    return {k: round(min(v, max_report), 2) for k, v in buckets.items()}


def format_scan(summary):
    """Sector dict -> one sentence of English."""
    if summary is None:
        return "LIDAR: no valid returns (sensor down, or fully occluded)."
    parts = [f"{k} {v}m" for k, v in sorted(summary.items(), key=lambda kv: kv[1])]
    nearest = min(summary.items(), key=lambda kv: kv[1])
    return (f"LIDAR nearest obstacle per sector: {', '.join(parts)}. "
            f"Closest is {nearest[1]}m to the {nearest[0]}.")


def has_room_to_turn(summary, direction):
    """Is there space for the 1.94m turning circle on that side?

    Returns True/False, or None if we cannot tell.
    """
    if summary is None:
        return None
    side = "left" if direction == "left" else "right"
    vals = [summary.get(side), summary.get(f"front-{side}")]
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return min(vals) >= MIN_TURN_RADIUS


# --- Failure translation ---------------------------------------------------
def diagnose_nav_failure(*, status, dist_remaining, dist_travelled,
                         stalled_seconds, elapsed, scan_summary,
                         had_global_plan, goal_xy,
                         commanded_yaw_at_low_speed, left_map=False):
    """Explain, in words, why a navigation attempt did not work.

    Every branch exists because it is a distinct failure that an agent would
    otherwise misread as something else. Adding branches here is how this
    project gets better.
    """
    lines = [
        f"navigate_to({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) ended: {status}.",
        f"Stopped {dist_remaining:.2f}m from the goal after {elapsed:.1f}s, "
        f"having travelled {dist_travelled:.2f}m.",
    ]

    if left_map:
        lines.append(
            "DIAGNOSIS: the robot drove beyond the edge of the mapped area "
            "during this attempt. Outside the costmap the planner cannot "
            "establish a valid start state, so every subsequent goal will "
            "fail until the robot reverses back into mapped space. This is "
            "not an obstacle problem.")

    if commanded_yaw_at_low_speed:
        lines.append(
            "DIAGNOSIS: rotation was commanded while nearly stationary. This "
            f"platform is a tricycle with a {MIN_TURN_RADIUS:.2f}m minimum "
            "turning radius and CANNOT rotate in place. It must be moving "
            "forward to change heading. Any plan that relies on turning on "
            "the spot will fail silently.")

    if not had_global_plan:
        lines.append(
            "DIAGNOSIS: the global planner never produced a path. The goal is "
            "likely unreachable, inside an obstacle, in unmapped space, or "
            "outside the costmap. Try a closer intermediate goal first.")
    elif stalled_seconds >= 3.0:
        lines.append(
            f"DIAGNOSIS: a path existed but the robot did not move for "
            f"{stalled_seconds:.1f}s. The local planner could not find a "
            "feasible trajectory. Given the wide turning circle this usually "
            "means the approach angle is too tight. Back off and re-approach "
            "from further out on a straighter line.")

    if scan_summary:
        nearest = min(scan_summary.items(), key=lambda kv: kv[1])
        if nearest[1] < 0.6:
            lines.append(
                f"DIAGNOSIS: obstacle {nearest[1]}m to the {nearest[0]} -- the "
                "robot is boxed in and needs clearance before any manoeuvre.")
        lines.append(format_scan(scan_summary))

    if dist_travelled < 0.1 and had_global_plan:
        lines.append(
            "NOTE: the robot never moved at all. Check the controller is "
            "publishing to /etrike/cmd_vel and cmd_vel_limiter is running.")

    return "\n".join(lines)


# --- Self-test -------------------------------------------------------------
if __name__ == "__main__":
    class FakeScan:
        """Minimal stand-in for sensor_msgs/LaserScan."""
        def __init__(self, ranges):
            self.ranges = ranges
            self.angle_min = -math.pi
            self.angle_increment = 2 * math.pi / len(ranges)
            self.range_min = 0.1
            self.range_max = 12.0

    print(f"Minimum turning radius: {MIN_TURN_RADIUS:.2f} m")
    print(f"Max yaw rate at 0.0 m/s: {max_yaw_rate(0.0):.3f} rad/s")
    print(f"Max yaw rate at 1.0 m/s: {max_yaw_rate(1.0):.3f} rad/s\n")

    # 360 rays, mostly clear, with a wall close in front.
    ranges = [8.0] * 360
    for i in range(170, 190):        # index 180 == 0 deg == straight ahead
        ranges[i] = 0.45
    for i in range(60, 90):
        ranges[i] = 2.2
    ranges[300] = float("inf")       # dropout, should be ignored

    summary = summarise_scan(FakeScan(ranges))
    print(format_scan(summary), "\n")
    print("Room to turn left? ", has_room_to_turn(summary, "left"))
    print("Room to turn right?", has_room_to_turn(summary, "right"), "\n")

    print("--- example diagnosis ---")
    print(diagnose_nav_failure(
        status="ABORTED", dist_remaining=2.31, dist_travelled=0.02,
        stalled_seconds=6.4, elapsed=18.2, scan_summary=summary,
        had_global_plan=True, goal_xy=(4.0, 1.5),
        commanded_yaw_at_low_speed=True))