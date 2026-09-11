#!/usr/bin/env python3
"""Four stationary pillar scenarios; planning only."""
import math
import time
import numpy as np
import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from nav2_msgs.action import ComputePathToPose
from gazebo_msgs.srv import GetModelList, DeleteEntity, SpawnEntity
from geometry import xf, poly_dist

FP = [(-.38,-.52),(1.63,-.52),(1.63,.52),(-.38,.52)]
CASES = [(1.6,0),(1.4,0),(1.6,15),(1.4,15)]

def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def main():
    rclpy.init()
    n = rclpy.create_node("pillar_batch_audit",parameter_overrides=[
        Parameter("use_sim_time",Parameter.Type.BOOL,True)])
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf,n)
    state,odom,results = {},[],[]

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)

    def wait(f,seconds=20):
        rclpy.spin_until_future_complete(n,f,timeout_sec=seconds)
        if not f.done():
            raise RuntimeError("Request timed out")
        return f.result()

    def call(kind,name,req):
        client = n.create_client(kind,name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError("Unavailable: "+name)
            return wait(client.call_async(req))
        finally:
            n.destroy_client(client)

    def odom_cb(m):
        v = m.twist.twist
        odom.append((time.monotonic(),v.linear.x,v.linear.y,v.angular.z))
        del odom[:-100]

    n.create_subscription(Odometry,"/tricycle_steering_controller/odometry",
                          odom_cb,qos_profile_sensor_data)
    n.create_subscription(Costmap,"/global_costmap/costmap_raw",
                          lambda m:state.update(grid=m),10)

    def pose():
        tf = buf.lookup_transform("map","base_link",Time())
        now = n.get_clock().now().nanoseconds/1e9
        age = now-Time.from_msg(tf.header.stamp).nanoseconds/1e9
        if now<=0 or not -.05<=age<=.5:
            raise RuntimeError("Clock/TF not ready")
        return tf.transform.translation.x,tf.transform.translation.y,\
               yaw(tf.transform.rotation)

    def stationary():
        recent = [v for v in odom if time.monotonic()-v[0]<1]
        return (len(recent)>=5 and recent[-1][0]-recent[0][0]>=.5
                and all(abs(x)<.01 and abs(y)<.01 and abs(w)<.02
                        for _,x,y,w in recent))

    def unchanged():
        x,y,a = pose()
        if not stationary() or math.hypot(x-sx,y-sy)>.02 or \
           abs(wrap(a-heading))>.01:
            raise RuntimeError("Robot/map pose changed; batch stopped")

    def grid_points(after):
        m = state.get("grid")
        if m is None or m.header.frame_id!="map":
            return None
        stamp = Time.from_msg(m.header.stamp).nanoseconds
        age = (n.get_clock().now().nanoseconds-stamp)/1e9
        if stamp<=after or not -.05<=age<=4:
            return None
        md = m.metadata
        g = np.asarray(m.data,dtype=np.uint8).reshape(md.size_y,md.size_x)
        jj,ii = np.nonzero(g==254)
        a = yaw(md.origin.orientation)
        u,v = (ii+.5)*md.resolution,(jj+.5)*md.resolution
        x = md.origin.position.x+math.cos(a)*u-math.sin(a)*v
        y = md.origin.position.y+math.sin(a)*u+math.cos(a)*v
        return x,y,md.resolution

    def counts_in_boxes(points,centres):
        x,y,res = points
        counts = []
        for bx,by,a in centres:
            u = (x-bx)*math.cos(a)+(y-by)*math.sin(a)
            v = -(x-bx)*math.sin(a)+(y-by)*math.cos(a)
            counts.append(int(((abs(u)<=.3+res)&(abs(v)<=.3+res)).sum()))
        return counts

    def wait_observations(after,centres,present):
        end = time.monotonic()+20
        while time.monotonic()<end:
            spin(.2)
            unchanged()
            points = grid_points(after)
            if points is None:
                continue
            counts = counts_in_boxes(points,centres)
            if (present and min(counts)>=4) or (not present and sum(counts)==0):
                return points[2],counts
        raise RuntimeError(
            "Pillars not detected" if present else
            "Old pillar cells did not clear; refusing a contaminated comparison")

    def map_pose(x,y,a):
        return (sx+math.cos(heading)*x-math.sin(heading)*y,
                sy+math.sin(heading)*x+math.cos(heading)*y,
                wrap(heading+a))

    def stamped(p):
        m = PoseStamped()
        m.header.frame_id="map"
        m.header.stamp=n.get_clock().now().to_msg()
        m.pose.position.x,m.pose.position.y=p[0],p[1]
        m.pose.orientation.z=math.sin(p[2]/2)
        m.pose.orientation.w=math.cos(p[2]/2)
        return m

    try:
        good=0
        end=time.monotonic()+20
        while time.monotonic()<end:
            spin(.2)
            try:
                pose()
                good=good+1 if stationary() else 0
            except Exception:
                good=0
            if good>=3:
                break
        if good<3:
            raise RuntimeError("Stationary robot/TF not ready")
        sx,sy,heading=pose()
        planner=ActionClient(n,ComputePathToPose,"/compute_path_to_pose")
        if not planner.wait_for_server(timeout_sec=10):
            raise RuntimeError("Planner unavailable")
        old_centres=None

        for gap,degrees in CASES:
            unchanged()
            models=call(GetModelList,"/get_model_list",GetModelList.Request())
            if not models.success or "bglx_etrike" not in models.model_names:
                raise RuntimeError("Trike unavailable")
            for name in ("box1","pillar_test_left","pillar_test_right"):
                if name in models.model_names:
                    req=DeleteEntity.Request()
                    req.name=name
                    response=call(DeleteEntity,"/delete_entity",req)
                    if not response.success:
                        raise RuntimeError(response.status_message)
            deleted=n.get_clock().now().nanoseconds
            if old_centres is not None:
                wait_observations(deleted,old_centres,False)
            else:
                spin(3)

            angle=math.radians(degrees)
            centres=[]
            for name,side in (
                ("pillar_test_left",gap/2+.3),
                ("pillar_test_right",-gap/2-.3)):
                unchanged()
                x=4-side*math.sin(angle)
                y=.5+side*math.cos(angle)
                req=SpawnEntity.Request()
                req.name=name
                req.reference_frame="bglx_etrike::base_link"
                req.initial_pose.position.x=x
                req.initial_pose.position.y=y
                req.initial_pose.position.z=.5
                req.initial_pose.orientation.z=math.sin(angle/2)
                req.initial_pose.orientation.w=math.cos(angle/2)
                req.xml=f"""<sdf version="1.6"><model name="{name}">
                <static>true</static><link name="body">
                <collision name="collision"><geometry>
                <box><size>0.6 0.6 1.0</size></box></geometry></collision>
                <visual name="visual"><geometry>
                <box><size>0.6 0.6 1.0</size></box></geometry>
                <material><ambient>1 0.4 0 1</ambient>
                <diffuse>1 0.4 0 1</diffuse></material></visual>
                </link></model></sdf>"""
                response=call(SpawnEntity,"/spawn_entity",req)
                if not response.success:
                    raise RuntimeError(response.status_message)
                centres.append(map_pose(x,y,angle))
            old_centres=centres
            resolution,counts=wait_observations(
                n.get_clock().now().nanoseconds,centres,True)
            boxes=[xf([(-.3,-.3),(.3,-.3),(.3,.3),(-.3,.3)],*p)
                   for p in centres]
            cx,cy,ch=map_pose(4,.5,angle)
            nx,ny=math.cos(ch),math.sin(ch)
            target=(cx+4*nx,cy+4*ny,ch)
            print(f"\nCASE gap={gap:.1f}m angle={degrees}deg "
                  f"offset=0.5m distance=4m grid={resolution:.3f}m "
                  f"cells={counts}",flush=True)

            for planner_id in ("GridBased","TightSpace"):
                unchanged()
                request=ComputePathToPose.Goal()
                request.planner_id=planner_id
                request.use_start=True
                request.start=stamped((sx,sy,heading))
                request.goal=stamped(target)
                ph=wait(planner.send_goal_async(request))
                if not ph.accepted:
                    results.append((gap,degrees,planner_id,"REJECTED"))
                    continue
                try:
                    result=wait(ph.get_result_async())
                except Exception:
                    wait(ph.cancel_goal_async(),5)
                    raise
                unchanged()
                path=result.result.path
                if result.status!=4 or len(path.poses)<2:
                    results.append((gap,degrees,planner_id,"NO PATH"))
                    continue
                if path.header.frame_id!="map":
                    raise RuntimeError("Unexpected path frame")
                poses=[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation))
                       for p in path.poses]
                minimum=float("inf")
                length=reverse=0.
                hits=0
                crossings=[]
                for (x,y,a),(xx,yy,aa) in zip(poses,poses[1:]):
                    dx,dy=xx-x,yy-y
                    ds=math.hypot(dx,dy)
                    da=wrap(aa-a)
                    length+=ds
                    if dx*math.cos(a)+dy*math.sin(a)<-1e-5:
                        reverse+=ds
                    f0,f1=(x-cx)*nx+(y-cy)*ny,(xx-cx)*nx+(yy-cy)*ny
                    if min(f0,f1)<0<=max(f0,f1):
                        t=-f0/(f1-f0)
                        crossings.append((
                            -(x+t*dx-cx)*ny+(y+t*dy-cy)*nx,
                            math.degrees(wrap(a+t*da-ch))))
                    steps=max(1,math.ceil((ds+1.72*abs(da))/.005))
                    for i in range(steps+1):
                        t=i/steps
                        body=xf(FP,x+t*dx,y+t*dy,a+t*da)
                        clr=min(poly_dist(body,b) for b in boxes)
                        minimum=min(minimum,clr)
                        hits+=clr<=0
                through=bool(crossings) and all(abs(y)<gap/2 for y,a in crossings)
                verdict=("PASS SCREEN" if through and hits==0 and minimum>=.15
                         else "INSUFFICIENT CLEARANCE" if through else "AROUND")
                detail=(f"{verdict}; min={minimum:.4f}m; length={length:.3f}m; "
                        f"reverse={reverse:.3f}m; intersections={hits}")
                results.append((gap,degrees,planner_id,detail))
                print(planner_id+": "+detail,flush=True)
                print("Crossings (offset m, heading error deg):",
                      [(round(y,3),round(a,2)) for y,a in crossings],flush=True)


            print("\nALIGNMENT FALLBACK CANDIDATES",flush=True)

            def stage_plan(planner_id,source_pose,target_pose):
                unchanged()
                request=ComputePathToPose.Goal()
                request.planner_id=planner_id
                request.use_start=True
                request.start=stamped(source_pose)
                request.goal=stamped(target_pose)
                ph=wait(planner.send_goal_async(request))
                if not ph.accepted:
                    return None
                try:
                    response=wait(ph.get_result_async())
                except Exception:
                    wait(ph.cancel_goal_async(),5)
                    raise
                unchanged()
                path=response.result.path
                if response.status!=4 or len(path.poses)<2:
                    return None
                if path.header.frame_id!="map":
                    raise RuntimeError("Unexpected stage path frame")
                return [(p.pose.position.x,p.pose.position.y,
                         yaw(p.pose.orientation)) for p in path.poses]

            def stage_audit(poses):
                minimum=float("inf")
                length=reverse=0.
                hits=0
                crossings=[]
                for (x,y,a),(xx,yy,aa) in zip(poses,poses[1:]):
                    dx,dy=xx-x,yy-y
                    ds=math.hypot(dx,dy)
                    da=wrap(aa-a)
                    length+=ds
                    if dx*math.cos(a)+dy*math.sin(a)<-1e-5:
                        reverse+=ds
                    f0=(x-cx)*nx+(y-cy)*ny
                    f1=(xx-cx)*nx+(yy-cy)*ny
                    if min(f0,f1)<0<=max(f0,f1):
                        t=-f0/(f1-f0)
                        crossings.append(-(x+t*dx-cx)*ny+(y+t*dy-cy)*nx)
                    steps=max(1,math.ceil((ds+1.72*abs(da))/.005))
                    for i in range(steps+1):
                        t=i/steps
                        body=xf(FP,x+t*dx,y+t*dy,a+t*da)
                        clr=min(poly_dist(body,b) for b in boxes)
                        minimum=min(minimum,clr)
                        hits+=clr<=0.
                return minimum,length,reverse,hits,crossings

            for setback in (2.5,3.0,3.5):
                label=f"Align{setback:.1f}"
                alignment=(cx-setback*nx,cy-setback*ny,ch)
                approach=stage_plan(
                    "TightSpace",(sx,sy,heading),alignment)
                if approach is None:
                    results.append((gap,degrees,label,"NO ALIGNMENT PATH"))
                    print(label+": no alignment path",flush=True)
                    continue

                amin,alen,arev,ahits,across=stage_audit(approach)
                endpoint=approach[-1]
                position_error=math.hypot(
                    endpoint[0]-alignment[0],endpoint[1]-alignment[1])
                heading_error=abs(wrap(endpoint[2]-ch))
                approach_ok=(
                    amin>=.15 and ahits==0 and not across
                    and position_error<=.05
                    and heading_error<=math.radians(1))
                if not approach_ok:
                    detail=(
                        f"ALIGNMENT REJECTED; min={amin:.4f}m; "
                        f"length={alen:.3f}m; reverse={arev:.3f}m; "
                        f"intersections={ahits}; "
                        f"endpoint_error={position_error:.3f}m/"
                        f"{math.degrees(heading_error):.2f}deg; "
                        f"premature_crossings={len(across)}")
                    results.append((gap,degrees,label,detail))
                    print(label+": "+detail,flush=True)
                    continue

                passage=stage_plan("GridBased",endpoint,target)
                if passage is None:
                    results.append((gap,degrees,label,"NO PASSAGE PATH"))
                    print(label+": no passage path",flush=True)
                    continue

                pmin,plen,prev,phits,pcross=stage_audit(passage)
                through=bool(pcross) and all(abs(y)<gap/2 for y in pcross)
                passed=pmin>=.15 and phits==0 and prev<.01 and through
                detail=(
                    f"{'PASS SCREEN' if passed else 'PASSAGE REJECTED'}; "
                    f"align_min={amin:.4f}m; passage_min={pmin:.4f}m; "
                    f"total_length={alen+plen:.3f}m; "
                    f"alignment_reverse={arev:.3f}m; "
                    f"passage_intersections={phits}; through={through}")
                results.append((gap,degrees,label,detail))
                print(label+": "+detail,flush=True)

    finally:
        print("\n========== BATCH SUMMARY ==========",flush=True)
        for gap,angle,planner_id,detail in results:
            print(f"gap={gap:.1f} angle={angle:2d} {planner_id}: {detail}",
                  flush=True)
        print("No movement requested. Last spawned scene remains.")
        print("Sampled audits use requested box placement and robot map pose.")
        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
