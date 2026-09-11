import math

def xf(poly, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c*u - s*v, y + s*u + c*v) for u, v in poly]

def edges(p): return zip(p, p[1:] + p[:1])

def intersects(a, b):
    for poly in (a, b):
        for p, q in edges(poly):
            nx, ny = -(q[1]-p[1]), q[0]-p[0]
            pa = [nx*x+ny*y for x, y in a]; pb = [nx*x+ny*y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa): return False
    return True

def pt_seg(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]; L = dx*dx+dy*dy
    if L == 0: return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/L))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

def poly_dist(a, b):
    if intersects(a, b): return 0.0
    return min(min(pt_seg(p,r,s), pt_seg(q,r,s), pt_seg(r,p,q), pt_seg(s,p,q))
               for p,q in edges(a) for r,s in edges(b))
