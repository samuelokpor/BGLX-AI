"""Will the vehicle physically fit through what is ahead?

Nav2 will happily plan through a gap its inflation layer tolerates and then
discover, mid-corridor, that the controller cannot follow the path. For a
differential-drive robot that is an inconvenience: it pivots and leaves. For
this tricycle it is a genuine failure. The vehicle is 1.55m long and 0.80m
wide, it cannot rotate in place, it needs roughly 1.5m of clear space to swing
round, and its LiDAR sees nothing behind it that the local costmap trusts.
Committing to a corridor it cannot fit is how the trike ends up stranded
somewhere a human has to go and fetch it.

So: measure the gap BEFORE entering it, and say plainly whether it fits, is
tight, or is impassable - and if impassable, say so while there is still room
to turn around.

The measurement is deliberately simple. Take the scan sector spanning the
direction of travel, walk outward from the centre bearing until the range
jumps (an opening) or closes in (a wall), and report the narrowest width the
vehicle would have to pass through.
"""

import math

VEHICLE_WIDTH = 0.80          # rear track plus tyre width
VEHICLE_LENGTH = 1.55         # -0.30 to 1.25 from base_footprint
MIRROR_CLEARANCE = 0.15       # each side: handlebars, cargo rails, wobble
TURN_AROUND_SPACE = 1.6       # roughly 2x minimum turning radius

# A gap must exceed this to be called comfortable rather than tight.
COMFORTABLE = VEHICLE_WIDTH + 2 * MIRROR_CLEARANCE + 0.40    # 1.50m
MINIMUM = VEHICLE_WIDTH + 2 * MIRROR_CLEARANCE               # 1.10m


def _rays(scan):
    """(bearing_deg, range) for every valid return, bearing wrapped."""
    out = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        out.append(((a + 180.0) % 360.0 - 180.0, r))
    return out


def _lateral_clearance(rays, centre_deg, look_ahead):
    """How far is the nearest obstacle to each side of the travel direction?

    Projects each return onto the axis perpendicular to travel. Only returns
    that lie WITHIN look_ahead metres along the direction of travel count -
    a wall 10m ahead does not constrain a 3m corridor.
    """
    left = right = None
    for bearing, r in rays:
        rel = math.radians(bearing - centre_deg)
        along = r * math.cos(rel)          # distance along travel
        across = r * math.sin(rel)         # +ve to the left
        if along < 0.1 or along > look_ahead:
            continue
        if across >= 0:
            left = across if left is None else min(left, across)
        else:
            right = -across if right is None else min(right, -across)
    return left, right


def check_corridor(scan, direction_deg=0.0, look_ahead=6.0):
    """Report whether the vehicle fits through the gap in that direction.

    direction_deg is relative to the robot's heading: 0 ahead, 180 behind.
    """
    if scan is None:
        return "Cannot check width: no LiDAR data."

    rays = _rays(scan)
    if not rays:
        return "Cannot check width: no valid LiDAR returns."

    left, right = _lateral_clearance(rays, direction_deg, look_ahead)
    where = "ahead" if abs(direction_deg) < 45 else (
        "behind" if abs(direction_deg) > 135 else
        ("to the left" if direction_deg > 0 else "to the right"))

    if left is None and right is None:
        return ("Open %s: nothing within %.0fm on either side of the path. "
                "Width is not a constraint here." % (where, look_ahead))

    # One-sided: an obstacle to pass rather than a corridor to thread.
    if left is None or right is None:
        side = "right" if left is None else "left"
        d = right if left is None else left
        if d < MIRROR_CLEARANCE + VEHICLE_WIDTH / 2:
            return ("TOO CLOSE %s: an obstacle only %.2fm to the %s, and this "
                    "vehicle is %.2fm wide. It will be struck. Move away from "
                    "that side before proceeding."
                    % (where, d, side, VEHICLE_WIDTH))
        return ("Clear %s: obstacle %.2fm to the %s, open on the other side. "
                "Passable." % (where, d, side, ))

    gap = left + right
    lines = ["Gap %s is %.2fm wide (%.2fm to the left, %.2fm to the right). "
             "The vehicle is %.2fm wide." % (where, gap, left, right,
                                             VEHICLE_WIDTH)]

    if gap < MINIMUM:
        lines.append(
            "WILL NOT FIT. %.2fm of clearance is needed including a margin "
            "each side, and only %.2fm is available. Do NOT enter. This "
            "platform cannot rotate in place and needs about %.1fm to turn "
            "around, so entering and discovering the problem later would "
            "leave it stuck. Find another route now, while there is still "
            "room to manoeuvre." % (MINIMUM, gap, TURN_AROUND_SPACE))
    elif gap < COMFORTABLE:
        off_centre = abs(left - right) / 2.0
        lines.append(
            "TIGHT but passable: %.2fm of margin in total. Approach "
            "square-on; an angled entry effectively needs more width because "
            "the vehicle is %.2fm long. " % (gap - VEHICLE_WIDTH,
                                             VEHICLE_LENGTH))
        if off_centre > 0.15:
            side = "left" if left > right else "right"
            lines.append("Currently off-centre by %.2fm - move %s before "
                         "entering." % (off_centre, side))
    else:
        lines.append("Comfortable: %.2fm of margin." % (gap - VEHICLE_WIDTH))
        off_centre = abs(left - right) / 2.0
        if off_centre > 0.30:
            side = "left" if left > right else "right"
            lines.append(
                "But the vehicle is %.2fm off-centre, closer to the %s wall. "
                "That matters even in a wide gap if the passage is long: this "
                "is a %.2fm vehicle that cannot correct sideways, so a small "
                "heading error compounds over the length of a tunnel. Centre "
                "up and approach square-on."
                % (off_centre, "right" if left > right else "left",
                   VEHICLE_LENGTH))
        else:
            lines.append("Passable at normal speed.")

    return " ".join(lines)


def can_turn_around(scan):
    """Is there room to reverse direction here?

    Worth asking before entering anywhere constrained, because the answer
    stops being yes once you are inside it.
    """
    if scan is None:
        return None, "Cannot tell: no LiDAR data."
    rays = _rays(scan)
    if not rays:
        return None, "Cannot tell: no valid LiDAR returns."

    # A turn needs clearance in a broad arc, not just straight ahead.
    worst = min(r for _, r in rays)
    if worst >= TURN_AROUND_SPACE:
        return True, ("Room to turn around: at least %.2fm clear in every "
                      "direction, and about %.1fm is needed."
                      % (worst, TURN_AROUND_SPACE))
    return False, ("NO ROOM TO TURN AROUND: the nearest obstacle is %.2fm "
                   "away and this vehicle needs about %.1fm of clear space to "
                   "swing round, because it cannot rotate in place. From here "
                   "it can only go forward or reverse along its current line."
                   % (worst, TURN_AROUND_SPACE))