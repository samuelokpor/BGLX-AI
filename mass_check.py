import math, sys, xml.etree.ElementTree as ET

def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]]

def mat_vec(m, v): return [sum(m[i][j]*v[j] for j in range(3)) for i in range(3)]
def mat_mul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def parse_origin(node):
    xyz, rpy = [0.0]*3, [0.0]*3
    if node is None: return xyz, rpy
    o = node.find('origin')
    if o is None: return xyz, rpy
    if o.get('xyz'): xyz = [float(v) for v in o.get('xyz').split()]
    if o.get('rpy'): rpy = [float(v) for v in o.get('rpy').split()]
    return xyz, rpy

def main(path, base='base_link'):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            links[link.get('name')] = (0.0, [0.0]*3); continue
        m = inertial.find('mass')
        mass = float(m.get('value')) if m is not None else 0.0
        xyz, _ = parse_origin(inertial)
        links[link.get('name')] = (mass, xyz)
    children = {}
    for joint in root.findall('joint'):
        p = joint.find('parent').get('link'); c = joint.find('child').get('link')
        xyz, rpy = parse_origin(joint)
        children.setdefault(p, []).append((c, xyz, rpy))
    IDENT = [[1,0,0],[0,1,0],[0,0,1]]; placed = {}
    def walk(link, pos, rot):
        placed[link] = (pos, rot)
        for c, jxyz, jrpy in children.get(link, []):
            jrot = rpy_to_matrix(*jrpy)
            cpos = [pos[i] + mat_vec(rot, jxyz)[i] for i in range(3)]
            walk(c, cpos, mat_mul(rot, jrot))
    if base not in links:
        print("base link not found"); return
    walk(base, [0.0]*3, IDENT)
    total = 0.0; moment = [0.0]*3; rows = []
    for name, (mass, ixyz) in links.items():
        if name not in placed or mass <= 0: continue
        pos, rot = placed[name]
        w = [pos[i] + mat_vec(rot, ixyz)[i] for i in range(3)]
        total += mass
        for i in range(3): moment[i] += mass * w[i]
        rows.append((mass, name, w))
    if total <= 0: print("no mass"); return
    com = [moment[i]/total for i in range(3)]
    print("\n=== MASS BUDGET (top 15 of %d links) ===" % len(rows))
    print("%9s  %-28s %s" % ("mass kg", "link", "position in base frame"))
    for mass, name, w in sorted(rows, reverse=True)[:15]:
        print("%9.3f  %-28s (%6.2f, %6.2f, %6.2f)" % (mass, name[:28], w[0], w[1], w[2]))
    print("\n=== TOTALS ===")
    print("total mass:      %.2f kg" % total)
    print("centre of mass:  (%.3f, %.3f, %.3f)" % tuple(com))
    print("CoM height:      %.3f m" % com[2])
    print("\n=== ROLLOVER ===")
    half = 0.55/2.0
    if com[2] > 0.01:
        a = 9.81*half/com[2]
        print("static rollover threshold: %.2f m/s^2 (%.2f g)" % (a, a/9.81))
        for v in (1.0, 1.5, 2.0, 2.78):
            print("  at %.2f m/s tightest safe radius: %.2f m" % (v, v*v/a))

main(sys.argv[1] if len(sys.argv) > 1 else '/tmp/etrike.urdf')
