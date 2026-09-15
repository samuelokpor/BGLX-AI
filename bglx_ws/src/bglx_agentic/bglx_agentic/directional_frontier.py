"""Bounded, goal-directed candidates. ROS-independent; Nav2 validates feasibility.

Grid search is candidate generation only, never a vehicle trajectory. A chosen
candidate is committed for a whole navigation action, preventing map-update
noise from changing the target on every callback.
"""
import heapq
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Grid:
    def __init__(self, msg, threshold):
        self.msg, self.threshold = msg, threshold
        self.r = msg.info.resolution
        self.w, self.h = msg.info.width, msg.info.height
        q = msg.info.origin.orientation
        self.angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.c, self.s = math.cos(self.angle), math.sin(self.angle)
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        if self.r <= 0 or self.w*self.h != len(msg.data):
            raise ValueError('Invalid grid')

    def index(self, x, y):
        dx, dy = x-self.ox, y-self.oy
        return (math.floor((self.c*dx+self.s*dy)/self.r),
                math.floor((-self.s*dx+self.c*dy)/self.r))

    def world(self, ix, iy):
        x, y = (ix+.5)*self.r, (iy+.5)*self.r
        return self.ox+self.c*x-self.s*y, self.oy+self.s*x+self.c*y

    def free(self, x, y):
        ix, iy = self.index(x, y)
        return (0 <= ix < self.w and 0 <= iy < self.h
                and 0 <= self.msg.data[iy*self.w+ix] < self.threshold)


def candidates(slam, costmap, pose, final, previous=None, horizon=12.0):
    """Return up to six spatially distinct observed-space targets.

    Connected grid search accommodates a corridor whose heading differs from
    the goal bearing. Direction penalties are preferences, not hard cones.
    No candidate behind the robot is selected merely to switch corridor sides.
    """
    a, b = Grid(slam, 65), Grid(costmap, 99)
    stride = max(1, math.ceil(.25/a.r))
    step = stride*a.r
    start = a.index(*pose[:2])
    goal_distance = math.dist(pose[:2], final[:2])
    bearing = math.atan2(final[1]-pose[1], final[0]-pose[0])
    cache = {}

    def free(cell):
        if cell not in cache:
            x, y = a.world(*cell)
            # Conservative square for candidate connectivity, not the trike
            # footprint. The orientation-aware full footprint is checked later.
            n = max(1, math.ceil(.335/min(a.r, b.r)))
            cache[cell] = all(
                a.free(x+.335*i/n, y+.335*j/n)
                and b.free(x+.335*i/n, y+.335*j/n)
                for i in range(-n, n+1) for j in range(-n, n+1))
        return cache[cell]

    if not free(start):
        return []
    distances = {start: 0.0}
    queue, scored = [(0.0, start)], []
    while queue and len(distances) < 18000:
        d, cell = heapq.heappop(queue)
        if d != distances[cell]:
            continue
        x, y = a.world(*cell)
        gain = goal_distance-math.hypot(final[0]-x, final[1]-y)
        heading = math.atan2(y-pose[1], x-pose[0])
        if d >= 3.0 and gain >= .4:
            score = (gain-.12*d
                     -.8*abs(wrap(heading-bearing))
                     -.6*abs(wrap(heading-pose[2])))
            if previous is not None:
                score -= .8*abs(wrap(heading-previous))
            scored.append((score, x, y, heading))
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),
                       (1,1),(1,-1),(-1,1),(-1,-1)):
            target = (cell[0]+dx*stride, cell[1]+dy*stride)
            nd = d+step*math.hypot(dx, dy)
            if nd > horizon or nd >= distances.get(target, math.inf):
                continue
            if not free(target):
                continue
            if dx and dy and (not free((target[0],cell[1]))
                              or not free((cell[0],target[1]))):
                continue
            # Cover the entire edge, not just its endpoint.
            tx, ty = a.world(*target)
            if not all(a.free(x+(tx-x)*k/stride, y+(ty-y)*k/stride)
                       and b.free(x+(tx-x)*k/stride, y+(ty-y)*k/stride)
                       for k in range(stride+1)):
                continue
            distances[target] = nd
            heapq.heappush(queue, (nd, target))
    result = []
    for _, x, y, heading in sorted(scored, reverse=True):
        if all(math.hypot(x-p[0], y-p[1]) >= 2.0 for p in result):
            result.append((x, y, heading))
            if len(result) == 6:
                break
    return result


def checked_path(path, slam, costmap):
    """Check dense full-footprint samples against observed SLAM and costmap.

    Includes each polygon's interior and a 5cm margin. SLAM occupied values
    are mapped to lethal values for the existing footprint-audit function.
    This is a snapshot preflight, not a substitute for live Nav2 checks.
    """
    from types import SimpleNamespace
    from .recovery_geometry import yaw
    if not path.poses or path.header.frame_id != slam.header.frame_id:
        return False, 'empty path / frame mismatch'
    if costmap.header.frame_id != path.header.frame_id:
        return False, 'costmap frame mismatch'
    mask = SimpleNamespace(info=slam.info, data=[
        -1 if v < 0 else 100 if v >= 65 else 0 for v in slam.data])
    samples = []
    for p in path.poses:
        v = p.pose
        point = (v.position.x, v.position.y, yaw(v.orientation))
        if not all(math.isfinite(t) for t in point):
            return False, 'nonfinite path'
        if samples:
            last = samples[-1]
            angle = wrap(point[2]-last[2])
            movement = math.dist(point[:2], last[:2])+1.2*abs(angle)
            count = max(1, math.ceil(movement/(min(slam.info.resolution,
                                                          costmap.info.resolution)/2)))
            if count > 1000:
                return False, 'discontinuous path'
            samples.extend((last[0]+(point[0]-last[0])*i/count,
                            last[1]+(point[1]-last[1])*i/count,
                            last[2]+angle*i/count) for i in range(1,count+1))
        else:
            samples.append(point)
    for pose in samples:
        for grid in (mask, costmap):
            ok, why = footprint_known(grid, pose)
            if not ok:
                return False, why
    return True, 'observed-space footprint preflight passed'


def footprint_known(grid, pose):
    """Same rectangle/SAT audit as recovery, avoiding SAT for free cells."""
    from .recovery_geometry import rectangle, overlaps, yaw
    info = grid.info
    r, w, h = info.resolution, info.width, info.height
    origin = info.origin.position
    angle = yaw(info.origin.orientation)
    c, s = math.cos(angle), math.sin(angle)
    poly = []
    for x, y in rectangle(pose, 0, (-.20, 1.04, .285), .05):
        dx, dy = x-origin.x, y-origin.y
        poly.append(((c*dx+s*dy)/r, (-s*dx+c*dy)/r))
    xs, ys = zip(*poly)
    if min(xs) <= 0 or min(ys) <= 0 or max(xs) >= w or max(ys) >= h:
        return False, 'footprint reaches map boundary'
    for iy in range(max(0,math.floor(min(ys))-1), min(h,math.floor(max(ys))+1)):
        for ix in range(max(0,math.floor(min(xs))-1), min(w,math.floor(max(xs))+1)):
            value = grid.data[iy*w+ix]
            if 0 <= value < 99:
                continue
            if overlaps(poly, [(ix,iy),(ix+1,iy),(ix+1,iy+1),(ix,iy+1)]):
                return False, 'footprint intersects unknown/occupied cell'
    return True, 'clear'
