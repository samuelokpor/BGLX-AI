"""Geometry shared by the concept scan filter, final guard and agent.

Pure functions intentionally have no ROS imports, so frame and clearance
regressions can be checked without a simulator.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np

FRONT_X = 1.63
REAR_X = -0.38
HALF_WIDTH = 0.52


def quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-10:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(q.x, q.y, q.z, q.w)
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite transform')
    return matrix


def origin_matrix(origin):
    result = np.eye(4)
    if origin is None:
        return result
    roll, pitch, yaw = map(float, origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]]
    result[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return result


def usable_scan(scan):
    if not scan.ranges or not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0:
        return False
    if not (0 <= scan.range_min < scan.range_max < float('inf')):
        return False
    return any((math.isfinite(r) and scan.range_min <= r <= scan.range_max)
               or r == float('inf') for r in scan.ranges)


def scan_points(scan, matrix):
    """Finite centre-row LaserScan endpoints in the target frame, with indices."""
    ranges = np.asarray(scan.ranges, dtype=float)
    indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max))
    angles = scan.angle_min + indices * scan.angle_increment
    r = ranges[indices]
    points = np.column_stack((r*np.cos(angles), r*np.sin(angles), np.zeros(len(indices))))
    return points @ matrix[:3, :3].T + matrix[:3, 3], indices


def corridor_clearance(points, forward=True, half_width=HALF_WIDTH):
    if not len(points):
        return None
    mask = (np.abs(points[:, 1]) <= half_width)
    mask &= points[:, 0] >= 0 if forward else points[:, 0] <= 0
    candidates = points[mask, 0]
    if not len(candidates):
        return None
    distance = candidates-FRONT_X if forward else REAR_X-candidates
    return max(0.0, float(np.min(distance)))


def collision_shapes(description):
    """Only supported physical collision primitives may be used for filtering."""
    shapes = []
    root = ET.fromstring(description)
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            if box is not None:
                kind, dimensions = 'box', np.array(list(map(float, box.get('size').split()))) / 2
            elif cylinder is not None:
                kind, dimensions = 'cylinder', (float(cylinder.get('radius')), float(cylinder.get('length'))/2)
            else:
                raise ValueError('Unsupported collision geometry; self filter cannot guess')
            shapes.append((link.get('name'), origin_matrix(collision.find('origin')), kind, dimensions))
    if not shapes:
        raise ValueError('No collision shapes in robot_description')
    return shapes


def self_mask(points_base, shapes, frames, padding=0.015):
    """Mask only endpoints inside actual collision solids (+15 mm noise margin).

    Never use the whole navigation footprint: open space between wheels may
    contain real obstacles. Removed beams become NaN, never free-space +inf.
    """
    mask = np.zeros(len(points_base), dtype=bool)
    for link, origin, kind, dimensions in shapes:
        transform = frames[link] @ origin
        local = (points_base-transform[:3, 3]) @ transform[:3, :3]
        if kind == 'box':
            inside = np.all(np.abs(local) <= dimensions+padding, axis=1)
        else:
            radius, half_length = dimensions
            # Simulation trial: rear-wheel self-return margin only.
            wheel_padding = (
                max(padding, 0.060)
                if link in ('rear_left_wheel', 'rear_right_wheel')
                else padding
            )
            inside = (
                (local[:, 0]**2 + local[:, 1]**2
                 <= (radius + wheel_padding)**2)
                & (np.abs(local[:, 2]) <= half_length + wheel_padding)
            )
        mask |= inside
    return mask
