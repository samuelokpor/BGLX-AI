"""Select nearby openings from current measurements, without site waypoints."""
import math


def evaluate(targets, robot, destination, max_range=7.0):
    rows = []
    forward = math.cos(robot[2]), math.sin(robot[2])
    for target in targets:
        mouth, normal, width = target['mouth'], target['normal'], target['width']
        dx, dy = mouth[0] - robot[0], mouth[1] - robot[1]
        distance = math.hypot(dx, dy)
        ahead = dx * forward[0] + dy * forward[1]
        before = dx * normal[0] + dy * normal[1]
        progress = ((destination[0] - mouth[0]) * normal[0]
                    + (destination[1] - mouth[1]) * normal[1])
        reason = None
        if not all(math.isfinite(v) for v in (*mouth, *normal, width, distance, progress)):
            reason = 'invalid geometry'
        elif not 1.0 <= width <= 2.1:
            reason = 'width outside supported passage range'
        elif not 2.0 <= distance <= max_range:
            reason = 'outside inspection distance'
        elif ahead < -.5:
            reason = 'behind current direction of travel'
        elif before < 1.5:
            reason = 'insufficient approach-side space'
        elif progress <= .5:
            reason = 'crossing normal points away from destination'
        rows.append(dict(target=target, eligible=reason is None, reason=reason,
                         distance_m=distance, ahead_m=ahead, destination_projection_m=progress))
    eligible = sorted((r for r in rows if r['eligible']), key=lambda r: (r['distance_m'], -r['target']['width']))
    return eligible, rows
