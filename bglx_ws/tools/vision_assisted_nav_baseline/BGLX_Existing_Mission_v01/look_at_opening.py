"""Short observation curve using the existing BGLX manoeuvre follower."""
import argparse
import hashlib
import importlib
import json
import math
import signal
import time
from pathlib import Path
import numpy as np

def arc(start, radius, turn):
    x,y,h = start
    length = radius*abs(turn)
    curvature = math.copysign(1/radius, turn)
    points = []
    for d in np.linspace(0, length, max(2, math.ceil(length/.02)+1)):
        a = h+curvature*d
        points.append((
            x+(math.sin(a)-math.sin(h))/curvature,
            y+(math.cos(h)-math.cos(a))/curvature, a
        ))
    x,y,h = points[-1]
    points.extend(
        (x+d*math.cos(h), y+d*math.sin(h), h)
        for d in np.linspace(.02, .65, 33)
    )
    return points

def project(point, pose, camera, k):
    x,y,h = pose
    c,s = math.cos(h), math.sin(h)
    dx,dy = point[0]-x, point[1]-y
    q = camera @ np.array([c*dx+s*dy, -s*dx+c*dy, point[2], 1.])
    if q[2] <= .1:
        return None
    uv = k @ q[:3]
    return uv[:2]/uv[2]

def choose(start, targets, camera, k, width, height, grid):
    choices = []
    footprint = np.array([
        [-.2,-.285], [1.04,-.285], [1.04,.285], [-.2,.285]
    ])
    for radius in (1.2, 1.6, 2., 2.5):
        for degrees in (-60,-45,-30,-20,20,30,45,60):
            turn = math.radians(degrees)
            length = radius*abs(turn)+.65
            if length > 2.5:
                continue
            points = arc(start, radius, turn)
            for target in targets:
                mouth = np.array(target["mouth"])
                normal = np.array(target["normal"])
                before = True
                for x,y,h in points:
                    c,s = math.cos(h), math.sin(h)
                    corners = footprint @ np.array([[c,s],[-s,c]]) + [x,y]
                    if np.max((corners-mouth[:2]) @ normal) > -.3:
                        before = False
                        break
                if not before:
                    continue
                visible = True
                for position in points[-19:]:
                    for error in (-.15, 0, .15):
                        uv = project(
                            mouth,
                            (position[0], position[1], position[2]+error),
                            camera, k
                        )
                        if uv is None or not (
                            .15*width < uv[0] < .85*width
                            and .1*height < uv[1] < .9*height
                        ):
                            visible = False
                            break
                    if not visible:
                        break
                if visible:
                    choices.append(
                        (length+.15*abs(turn), points, target, degrees)
                    )
    for _,points,target,degrees in sorted(choices, key=lambda item:item[0]):
        if grid.path_clear(points):
            return points,target,degrees
    return None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    expected = json.loads((here/"source_hashes.json").read_text())
    for name,digest in expected.items():
        module = importlib.import_module("bglx_agentic."+name)
        actual = hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
        if actual != digest:
            raise RuntimeError("Unreviewed module change: "+name)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, LaserScan
    from bglx_agentic.delivery_mission import DeliveryMission
    from bglx_agentic.passage_geometry import Grid
    from openings import find_openings
    from analyze import matrix

    rclpy.init(
        args=["--ros-args", "-p", "use_sim_time:=true"],
        signal_handler_options=SignalHandlerOptions.NO
    )
    def interrupted(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    node = DeliveryMission()
    guard = node.recovery_guard
    data = {}
    success = False
    stopped = False
    subscriptions = [
        node.create_subscription(
            typ, topic,
            lambda msg,key=key: data.__setitem__(key,(msg,time.monotonic())),
            qos_profile_sensor_data
        )
        for typ,topic,key in [
            (CameraInfo, "/etrike/front/camera_info", "info"),
            (LaserScan, "/etrike/scan", "scan")
        ]
    ]
    try:
        guard.ready()
        guard.spin(1.)
        def fresh(key):
            msg,receipt = data.get(key,(None,0))
            if msg is None or time.monotonic()-receipt > .75:
                raise RuntimeError("Missing/stale "+key)
            age = node.get_clock().now().nanoseconds/1e9 - (
                msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            )
            if not -.1 <= age <= .75:
                raise RuntimeError("Stale timestamp "+key)
            return msg

        info,scan = fresh("info"),fresh("scan")
        if any(abs(v)>1e-8 for v in info.d):
            raise RuntimeError("Camera must be rectified")
        grid_msg = guard.fresh("grid",.75)
        frame = grid_msg.header.frame_id
        start = guard.transform(frame,"base_footprint")

        def tf(target,source):
            guard.transform(target,source)
            transform = node.tf_buffer.lookup_transform(
                target,source,Time()
            )
            return matrix(transform.transform)

        lidar = tf(frame,scan.header.frame_id)
        base_z = tf(frame,"base_footprint")[2,3]
        camera = tf(info.header.frame_id,"base_footprint")
        k = np.asarray(info.k).reshape(3,3)
        targets = []
        for gap in find_openings(
            scan.ranges,scan.angle_min,scan.angle_increment,
            scan.range_min,scan.range_max
        ):
            if not 1.0 <= gap["width_at_scan_height_m"] <= 2.1:
                continue
            mouth = lidar @ np.array([*gap["mouth_scan_xy"],0.,1.])
            mouth[2] -= base_z
            normal = lidar[:2,:2] @ np.array(gap["outward_normal_scan_xy"])
            normal /= np.linalg.norm(normal)
            if math.dist(start[:2],mouth[:2]) > 8:
                continue
            targets.append(dict(
                mouth=mouth[:3].tolist(), normal=normal.tolist(),
                width=gap["width_at_scan_height_m"]
            ))

        ok,reason = guard.reverse_audit(0.)
        if not ok:
            raise RuntimeError("Existing guard: "+str(reason))
        selected = choose(
            start,targets,camera,k,info.width,info.height,Grid(grid_msg)
        )
        if selected is None:
            raise RuntimeError(
                "No short, footprint-clear viewing curve found; no motion"
            )
        points,target,degrees = selected
        out = Path.home()/"bglx_navtest/logs"/("look_"+str(time.time_ns()))
        out.mkdir(parents=True)
        (out/"look.json").write_text(json.dumps(dict(
            frame=frame,points=points,opening=target,
            turn_degrees=degrees,execute=args.execute
        ),indent=2))
        node.passage_alignment.debug.publish(
            node.passage_alignment.make_path(points,frame)
        )
        print("LOOK PLAN:",json.dumps(dict(
            target=target,end=points[-1],
            turn_degrees=degrees,speed_mps=.25
        )),flush=True)
        print("RESULTS:",out,flush=True)

        if args.execute:
            success = node.passage_alignment.follow_checked(
                points,frame,time.monotonic()+45,points[-1][2]
            )
            if not success:
                raise RuntimeError("Observation manoeuvre failed")
            guard.spin(.2)
            actual = guard.transform(frame,"base_footprint")
            uv = project(target["mouth"],actual,camera,k)
            if uv is None or not (
                0 < uv[0] < info.width and 0 < uv[1] < info.height
            ):
                raise RuntimeError("Target still outside predicted camera view")
            print("LOOK COMPLETE: ready for fresh vision selection.",flush=True)
        else:
            success = True
            print("LOOK PLAN PASSED. No movement requested.",flush=True)
    except (Exception,KeyboardInterrupt) as error:
        success = False
        print("LOOK ABORT:",str(error),flush=True)
    finally:
        stopped = bool(guard.cleanup())
        stopped = guard.stopped() and stopped
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if success and stopped else 1

if __name__ == "__main__":
    raise SystemExit(main())
