"""Local passage geometry. No Gazebo names, campus coordinates or motion IO.

The Hermite connector follows the former supervised-gap approach: align
before the opening, then cross on a straight tangent. OccupancyGrid costs
are -1..100, not raw Nav2 costs. Unknown cells are rejected, but a costmap
configured to turn unknown into free cannot establish observed free space.
"""
import math
from dataclasses import dataclass

import numpy as np

from .recovery_geometry import yaw


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def xf(p, t):
    c, s = math.cos(t[2]), math.sin(t[2])
    return (t[0]+c*p[0]-s*p[1], t[1]+s*p[0]+c*p[1], wrap(p[2]+t[2]))


@dataclass(frozen=True)
class Gate:
    x: float
    y: float
    heading: float
    width: float

    def at(self, distance):
        return (self.x+distance*math.cos(self.heading),
                self.y+distance*math.sin(self.heading), self.heading)


class Grid:
    def __init__(self, msg):
        info = msg.info
        self.r, self.w, self.h = info.resolution, info.width, info.height
        self.origin = (info.origin.position.x, info.origin.position.y,
                       yaw(info.origin.orientation))
        if (not all(math.isfinite(v) for v in (*self.origin, self.r)) or self.r <= 0
                or self.w <= 0 or self.h <= 0 or len(msg.data) != self.w*self.h):
            raise ValueError('invalid local costmap')
        self.a = np.asarray(msg.data, dtype=np.int16).reshape(self.h, self.w)
        yy, xx = np.nonzero((self.a < 0) | (self.a >= 99))
        self.blocked = np.column_stack(((xx+.5)*self.r, (yy+.5)*self.r))

    def local(self, p):
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        x, y = p[0]-self.origin[0], p[1]-self.origin[1]
        return c*x+s*y, -s*x+c*y

    def cost(self, x, y):
        gx, gy = self.local((x, y))
        ix, iy = math.floor(gx/self.r), math.floor(gy/self.r)
        return int(self.a[iy, ix]) if 0 <= ix < self.w and 0 <= iy < self.h else -1

    def footprint(self, pose, margin=.15):
        """Exact rectangle/cell SAT, including interiors and touching edges."""
        x, y = self.local(pose)
        a = pose[2]-self.origin[2]
        c, s = math.cos(a), math.sin(a)
        # Asymmetric body [-.20,1.04] x [-.285,.285].
        cx, cy = x+.42*c, y+.42*s
        hx, hy = .62+margin, .285+margin
        ex, ey = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
        if cx-ex <= 0 or cy-ey <= 0 or cx+ex >= self.w*self.r or cy+ey >= self.h*self.r:
            return False
        if not len(self.blocked):
            return True
        delta = self.blocked - (cx, cy)
        half = self.r/2
        candidate = (np.abs(delta[:, 0]) <= ex+half) & (np.abs(delta[:, 1]) <= ey+half)
        delta = delta[candidate]
        if not len(delta):
            return True
        u = delta[:, 0]*c+delta[:, 1]*s
        v = -delta[:, 0]*s+delta[:, 1]*c
        cell_projection = half*(abs(c)+abs(s))
        return not bool(np.any((np.abs(u) <= hx+cell_projection+1e-9)
                               & (np.abs(v) <= hy+cell_projection+1e-9)))

    def path_clear(self, points, margin=.15):
        # Densify translation AND rotation; add half a sample of swept padding.
        for p, q in zip(points, points[1:]):
            ds, da = math.dist(p[:2], q[:2]), wrap(q[2]-p[2])
            count = max(1, math.ceil((ds+1.3*abs(da))/.025))
            for i in range(count+1):
                t = i/count
                pose = (p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1]), p[2]+t*da)
                if not self.footprint(pose, margin+.013):
                    return False
        return len(points) >= 2

    def section(self, p, max_side=1.4):
        """Find solid surfaces on both sides of this path cross-section."""
        if self.cost(*p[:2]) < 0 or self.cost(*p[:2]) >= 99:
            return None
        nx, ny = -math.sin(p[2]), math.cos(p[2])
        hits = []
        for sign in (-1, 1):
            hit = None
            for d in np.arange(.025, max_side+.001, min(.025, self.r/2)):
                value = self.cost(p[0]+sign*d*nx, p[1]+sign*d*ny)
                if value < 0:
                    return None
                if value == 100:
                    hit = float(d)
                    break
            hits.append(hit)
        if any(h is None for h in hits):
            return None
        right, left = hits
        offset = (left-right)/2
        return Gate(p[0]+offset*nx, p[1]+offset*ny, p[2], left+right)


def candidate_gate(grid, points, robot):
    """Nearest localized opening on Nav2's chosen path; never choose a new route."""
    if len(points) < 3:
        return None
    nearest = min(range(len(points)), key=lambda i: math.dist(points[i][:2], robot[:2]))
    distance, next_sample = 0.0, 2.5
    candidates = []
    for p, q in zip(points[nearest:], points[nearest+1:]):
        ds = math.dist(p[:2], q[:2])
        if ds < 1e-6:
            continue
        while next_sample <= distance+ds:
            t = (next_sample-distance)/ds
            a = math.atan2(q[1]-p[1], q[0]-p[0])
            sample = (p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1]), a)
            gate = grid.section(sample)
            if gate and 1.0 <= gate.width <= 2.1:
                before, after = grid.section(gate.at(-.8)), grid.section(gate.at(.8))
                # Do not turn every long corridor into an alignment exercise.
                if (before is None or before.width > gate.width+.25) and (
                        after is None or after.width > gate.width+.25):
                    along = ((gate.x-robot[0])*math.cos(a)+(gate.y-robot[1])*math.sin(a))
                    if along > 2.0 and abs(wrap(robot[2]-a)) < math.radians(80):
                        candidates.append((next_sample, gate))
            next_sample += .15
            if next_sample > 5.5:
                break
        if next_sample > 5.5:
            break
        distance += ds
    if not candidates:
        return None
    first_distance = candidates[0][0]
    # Select the throat of the first opening, not a later, narrower opening.
    return min((g for d, g in candidates if d < first_distance+.65), key=lambda g: g.width)


def hermite(start, end, scale):
    distance = math.dist(start[:2], end[:2])
    tangent = scale*distance
    m0 = tangent*np.array((math.cos(start[2]), math.sin(start[2])))
    m1 = tangent*np.array((math.cos(end[2]), math.sin(end[2])))
    t = np.linspace(0, 1, max(200, math.ceil((distance+2*tangent)/.008)))[:, None]
    p0, p1 = np.array(start[:2]), np.array(end[:2])
    p = (2*t**3-3*t*t+1)*p0+(-2*t**3+3*t*t)*p1+(t**3-2*t*t+t)*m0+(t**3-t*t)*m1
    d = (6*t*t-6*t)*p0+(-6*t*t+6*t)*p1+(3*t*t-4*t+1)*m0+(3*t*t-2*t)*m1
    dd = (12*t-6)*p0+(-12*t+6)*p1+(6*t-4)*m0+(6*t-2)*m1
    speed2 = np.sum(d*d, axis=1)
    if np.min(speed2) < 1e-8:
        return None
    k = np.abs(d[:, 0]*dd[:, 1]-d[:, 1]*dd[:, 0])/speed2**1.5
    headings = np.arctan2(d[:, 1], d[:, 0])
    if np.max(k) > 1/.75 or any(abs(wrap(a-end[2])) > math.radians(80) for a in headings):
        return None
    return [(float(x), float(y), float(a)) for (x, y), a in zip(p, headings)]


def aligned_path(grid, start, gate):
    for setback in (1.7, 2.1, 1.4):
        target = gate.at(-setback)
        if math.dist(start[:2], target[:2]) < .35:
            continue
        for scale in (1., 1.4, .75, 1.8):
            curve = hermite(start, target, scale)
            if curve is None:
                continue
            # Complete the crossing with the rear of the body clear of the throat.
            straight = [gate.at(float(d)) for d in np.linspace(-setback, 1.5, 160)]
            points = curve + straight[1:]
            # Bound local excursions and avoid long looping connectors.
            if sum(math.dist(p[:2], q[:2]) for p, q in zip(points, points[1:])) > 12:
                continue
            if grid.path_clear(points):
                return points
    return None
