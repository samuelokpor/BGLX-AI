"""Conservative, ROS-independent straight reverse footprint audit.

OccupancyGrid values are -1..100, NOT raw Nav2 0..255 costs. The
inscribed-cost value 99 is rejected as well as lethal 100. All cells
intersecting the swept rectangle are tested, including its interior.
"""
import math


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y),
                      1 - 2 * (q.y*q.y + q.z*q.z))


def rectangle(pose, distance, bounds, margin):
    x, y, angle = pose
    rear, front, half_width = bounds
    c, s = math.cos(angle), math.sin(angle)
    return [(x+c*u-s*v, y+s*u+c*v) for u, v in (
        (rear-distance-margin, -half_width-margin),
        (front+margin, -half_width-margin),
        (front+margin, half_width+margin),
        (rear-distance-margin, half_width+margin))]


def overlaps(a, b):
    # Separating-axis theorem; touching cells count as occupied.
    for polygon in (a, b):
        for i, p in enumerate(polygon):
            q = polygon[(i+1) % len(polygon)]
            axis = (p[1]-q[1], q[0]-p[0])
            pa = [v[0]*axis[0] + v[1]*axis[1] for v in a]
            pb = [v[0]*axis[0] + v[1]*axis[1] for v in b]
            if max(pa) < min(pb)-1e-10 or max(pb) < min(pa)-1e-10:
                return False
    return True


def audit(grid, pose, distance, bounds=(-0.20, 1.04, 0.285), margin=0.05):
    """pose is the base pose in grid.header.frame_id. Return (ok, reason)."""
    info = grid.info
    r, width, height = info.resolution, info.width, info.height
    if (not math.isfinite(r) or r <= 0 or width <= 0 or height <= 0
            or len(grid.data) != width*height
            or not all(math.isfinite(v) for v in (*pose, distance, margin))
            or distance < 0 or margin < 0):
        return False, 'invalid geometry or costmap'
    origin = info.origin
    angle = yaw(origin.orientation)
    c, s = math.cos(angle), math.sin(angle)
    poly = []
    for x, y in rectangle(pose, distance, bounds, margin):
        dx, dy = x-origin.position.x, y-origin.position.y
        poly.append(((c*dx+s*dy)/r, (-s*dx+c*dy)/r))
    if not all(math.isfinite(v) for p in poly for v in p):
        return False, 'nonfinite grid transform'
    xs, ys = list(zip(*poly))
    if min(xs) <= 0 or min(ys) <= 0 or max(xs) >= width or max(ys) >= height:
        return False, 'swept footprint outside local costmap'
    for iy in range(math.floor(min(ys))-1, math.floor(max(ys))+1):
        for ix in range(math.floor(min(xs))-1, math.floor(max(xs))+1):
            cell = [(ix, iy), (ix+1, iy), (ix+1, iy+1), (ix, iy+1)]
            if not overlaps(poly, cell):
                continue
            if not (0 <= ix < width and 0 <= iy < height):
                return False, 'swept footprint touches grid boundary'
            value = grid.data[iy*width+ix]
            if value < 0 or value >= 99:
                return False, 'unknown/occupied swept cell (%d,%d), cost=%d' % (ix, iy, value)
    return True, 'full reverse footprint clear'
