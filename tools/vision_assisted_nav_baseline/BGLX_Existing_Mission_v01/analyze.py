#!/usr/bin/env python3
"""Offline, read-only rosbag analysis; never publishes ROS messages."""
import argparse,json,sqlite3,math
from pathlib import Path
import numpy as np
import cv2

def timestamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9

def matrix(t):
 q=t.rotation;x,y,z,w=q.x,q.y,q.z,q.w
 n=x*x+y*y+z*z+w*w
 if abs(n-1)>0.01:raise ValueError('Invalid TF quaternion')
 a=np.eye(4);a[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
 a[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return a

def main():
 p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 dbs=list(args.bag.glob('*.db3'))
 if len(dbs)!=1:raise SystemExit('Expected one SQLite bag segment')
 try:
  from rclpy.serialization import deserialize_message
  from rosidl_runtime_py.utilities import get_message
  decode=lambda b,t:deserialize_message(b,get_message(t))
 except ImportError:
  from rosbags.typesys import Stores,get_typestore
  store=get_typestore(Stores.ROS2_HUMBLE);decode=store.deserialize_cdr
 db=sqlite3.connect('file:'+str(dbs[0].resolve())+'?mode=ro',uri=True)
 topics={name:(i,t) for i,name,t in db.execute('select id,name,type from topics')}
 def messages(name):
  i,t=topics[name]
  return [(stamp,decode(b,t)) for stamp,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
 # BGLX synchronized-frame selection v1
 i,typ=topics['/etrike/front/image_raw'];n=db.execute('select count(*) from messages where topic_id=?',(i,)).fetchone()[0]
 if n == 0:raise ValueError('No recorded RGB frames')
 from collections import Counter
 import time as sync_time
 read_messages=messages
 sync_cache={}
 def messages(name):
  if name not in sync_cache:sync_cache[name]=read_messages(name)
  return sync_cache[name]
 sync_errors=Counter();sync_selected=False;sync_attempted=0
 sync_deadline=sync_time.monotonic()+25.0
 for frame_index in sorted(range(n),key=lambda j:abs(j-n//2)):
  if sync_time.monotonic()>sync_deadline:break
  sync_attempted+=1
  try:
   receipt,b=db.execute('select timestamp,data from messages where topic_id=? order by timestamp limit 1 offset ?',(i,frame_index)).fetchone()
   image=decode(b,typ);st=timestamp(image)
   def near(name,age):
    _,m=min(messages(name),key=lambda x:abs(timestamp(x[1])-st))
    if abs(timestamp(m)-st)>age:raise ValueError('No sufficiently close '+name)
    return m
   info=near('/etrike/front/camera_info',.075);scan=near('/etrike/scan',.15)
   grid=near('/map',2.5);cost=near('/global_costmap/costmap',2.5)
   odom=near('/tricycle_steering_controller/odometry',.1)
   if abs(odom.twist.twist.linear.x)>.03 or abs(odom.twist.twist.angular.z)>.03:raise ValueError('Stationary capture required')
   if image.header.frame_id!=info.header.frame_id or (image.width,image.height)!=(info.width,info.height):raise ValueError('Camera calibration mismatch')
   if any(abs(v)>1e-8 for v in info.d):raise ValueError('Rectification required')
   edges={}
   for _,msg in messages('/tf_static'):
    for t in msg.transforms:edges[(t.header.frame_id,t.child_frame_id)]=matrix(t.transform)
   dynamic={}
   for _,msg in messages('/tf'):
    for t in msg.transforms:dynamic.setdefault((t.header.frame_id,t.child_frame_id),[]).append(t)
   for key,values in dynamic.items():
    t=min(values,key=lambda t:abs(timestamp(t)-st))
    # Future-dated map TF is permitted only within recorded coverage; interpolate.
    before=[t for t in values if timestamp(t)<=st];after=[t for t in values if timestamp(t)>=st]
    if not before or not after:continue
    a=max(before,key=timestamp);b=min(after,key=timestamp)
    if timestamp(b)-timestamp(a)>.3:continue
    A=matrix(a.transform);B=matrix(b.transform)
    f=(st-timestamp(a))/max(timestamp(b)-timestamp(a),1e-9)
    # Polar decomposition maintains orthonormality for short stationary samples.
    T=A*(1-f)+B*f;u,_,vh=np.linalg.svd(T[:3,:3]);T[:3,:3]=u@vh;T[3]=[0,0,0,1];edges[key]=T
   def transform(target,source):
    todo=[(source,np.eye(4))];seen=set()
    while todo:
     frame,T=todo.pop()
     if frame==target:return T
     if frame in seen:continue
     seen.add(frame)
     for (parent,child),M in edges.items():
      if frame==child:todo.append((parent,M@T))
      if frame==parent:todo.append((child,np.linalg.inv(M)@T))
    raise ValueError('Missing timestamp-supported TF '+source+' -> '+target)
   base=transform('map','base_footprint');cam=transform(image.header.frame_id,'map')
  except ValueError as error:
   reason=str(error)
   if not reason.startswith(('Missing timestamp-supported TF ', 'No sufficiently close ')):
    raise
   sync_errors[reason]+=1
   continue
  sync_selected=True
  print('SYNCHRONIZED FRAME:',json.dumps(dict(index=frame_index,total=n,
        attempts=sync_attempted,stamp=st,skipped=dict(sync_errors))),flush=True)
  break
 messages=read_messages
 sync_cache.clear()
 if not sync_selected:
  print('SYNC FAILURES:',json.dumps(dict(sync_errors)),flush=True)
  for key,values in locals().get('dynamic',{}).items():
   if key in locals().get('edges',{}):continue
   stamps=[timestamp(t) for t in values]
   before=max((s for s in stamps if s<=st),default=None)
   after=min((s for s in stamps if s>=st),default=None)
   print('UNAVAILABLE TF:',json.dumps(dict(frames=key,image_stamp=st,
         before=before,after=after,samples=len(stamps))),flush=True)
  raise ValueError('No synchronized RGB/TF sample found in %d attempts; no extrapolation used'%sync_attempted)
 yaw=math.atan2(base[1,0],base[0,0]);origin=base[:2,3]
 if grid.header.frame_id!='map' or cost.header.frame_id!='map':raise ValueError('Expected map-frame grids')
 def prepare(g):
  q=g.info.origin.orientation
  if abs(q.x)+abs(q.y)+abs(q.z)>1e-6:raise ValueError('Rotated grid unsupported')
  arr=np.asarray(g.data).reshape(g.info.height,g.info.width)
  free=(arr>=0)&(arr<20)
  free=np.pad(free.astype(np.uint8),1)
  clearance=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*g.info.resolution
  return g,clearance
 grids=[prepare(grid),prepare(cost)]
 # Use actual uploaded prototype's full circumscribed footprint plus margin.
 radius=math.hypot(1.04,.285)+.15
 def clear(xy):
  for g,d in grids:
   ix=int(math.floor((xy[0]-g.info.origin.position.x)/g.info.resolution));iy=int(math.floor((xy[1]-g.info.origin.position.y)/g.info.resolution))
   if not (0<=ix<g.info.width and 0<=iy<g.info.height) or d[iy,ix]<radius+g.info.resolution:return False
  return True
 from depth_evidence import inspect
 depth_reports=inspect(near,transform,topics,st)
 # Wall recesses require supported jambs and measured returns beyond the mouth.
 from openings import find_openings
 lidar=transform('map',scan.header.frame_id)
 candidates=[];all_openings=[]
 k=np.asarray(info.k).reshape(3,3)
 for gap in find_openings(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max):
  jambs=[lidar@np.array([*xy,0.,1.]) for xy in gap['jambs_scan_xy']]
  mouth=(jambs[0]+jambs[1])/2
  pt=cam@mouth;pix=k@pt[:3]
  uv=(pix[:2]/pix[2]).tolist() if pt[2]>1e-6 else None
  visible=bool(uv and 0<uv[0]<image.width and 0<uv[1]<image.height)
  all_openings.append(dict(
   mouth_map_xy=mouth[:2].tolist(),
   width_m=gap['width_at_scan_height_m'],
   visible=visible,pixel=uv,
   reason='inside camera' if visible else (
    'behind camera' if pt[2]<=0 else 'outside image')))
  if pt[2]<=0:continue
  u,v=pix[:2]/pix[2]
  if not(0<u<image.width and 0<v<image.height):continue
  normal=lidar[:2,:2]@np.array(gap['outward_normal_scan_xy'])
  approach=mouth[:2]-2.0*normal
  endpoint_clear=clear(approach)
  pixels=[]
  for jamb in jambs:
   c=cam@jamb;q=k@c[:3];pixels.append((q[:2]/q[2]).tolist())
  candidates.append(dict(id='O'+str(len(candidates)+1),**gap,
   mouth_map_xy=mouth[:2].tolist(),jambs_map_xy=[j[:2].tolist() for j in jambs],
   pixel=[int(u),int(v)],jamb_pixels=pixels,approach_map_xy=approach.tolist(),
   approach_yaw=math.atan2(normal[1],normal[0]),approach_endpoint_clear=endpoint_clear,
   kind='scan-height opening hypothesis',ground_clearance='unknown: local depth summary does not certify opening',
   connecting_path_checked=False,navigation_approved=False))
 # Proper row stride handling.
 channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4}
 if image.encoding not in channels:raise ValueError('Unsupported image encoding')
 ch=channels[image.encoding];raw=np.frombuffer(image.data,dtype=np.uint8).reshape(image.height,image.step)[:,:image.width*ch].reshape(image.height,image.width,ch)
 bgr=raw[:,:,:3].copy()
 if image.encoding.startswith('rgb'):bgr=bgr[:,:,::-1].copy()
 overlay=bgr.copy()
 for c in candidates:
  for jp in c['jamb_pixels']:
   ju,jv=map(int,jp);cv2.line(overlay,(ju,max(0,jv-65)),(ju,min(image.height-1,jv+20)),(0,255,255),2)
  u,v=c['pixel'];cv2.circle(overlay,(u,v),12,(0,255,255),2);cv2.putText(overlay,c['id'],(u-15,v-18),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
 # Scan returns projected at their actual measured height, not camera ground pixels.
 lidar=transform('map',scan.header.frame_id);points=[]
 for j,r in enumerate(scan.ranges):
  if not np.isfinite(r) or not scan.range_min<r<scan.range_max:continue
  a=scan.angle_min+j*scan.angle_increment;pt=cam@lidar@np.array([r*math.cos(a),r*math.sin(a),0,1])
  if pt[2]<=0:continue
  pix=np.asarray(info.k).reshape(3,3)@pt[:3];u,v=pix[:2]/pix[2]
  if 0<=u<image.width and 0<=v<image.height:points.append([int(u),int(v)])
 lidar_overlay=bgr.copy()
 for u,v in points:cv2.circle(lidar_overlay,(u,v),2,(0,0,255),-1)
 cv2.putText(overlay,'WALL OPENING HYPOTHESES - GROUND / PATH UNVERIFIED',(8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
 args.output.mkdir(parents=True,exist_ok=False)
 # Save calibrated depth for approach-stage flat-floor evidence checks.
 for topic,maximum in [('/etrike/front_camera/depth/image_raw',6.),('/etrike/front_depth/depth/image_raw',4.)]:
  if topic not in topics:continue
  from depth_evidence import decode_depth
  dm=near(topic,.15);di=near(topic.replace('image_raw','camera_info'),.15)
  if (dm.width,dm.height,dm.header.frame_id)!=(di.width,di.height,di.header.frame_id):raise ValueError('Depth calibration mismatch')
  if any(abs(v)>1e-8 for v in di.d):raise ValueError('Unrectified depth')
  depth,valid=decode_depth(dm,maximum)
  np.savez_compressed(args.output/(topic.split('/')[2]+'_depth.npz'),depth=depth,valid=valid,K=np.asarray(di.k).reshape(3,3),camera_from_map=transform(dm.header.frame_id,'map'))

 for name,im in [('rgb',bgr),('candidates',overlay),('lidar_projection',lidar_overlay)]:
  if not cv2.imwrite(str(args.output/(name+'.jpg')),im):raise RuntimeError('Save failed')
 arr=np.asarray(grid.data)
 map_stats=dict(total_cells=int(arr.size),unknown_cells=int((arr<0).sum()),free_cells=int((arr==0).sum()),occupied_cells=int((arr>=65).sum()))
 report=dict(all_openings=all_openings,depth_evidence=depth_reports,map_stats=map_stats,image_stamp=st,robot_map_xy=origin.tolist(),robot_yaw=yaw,candidates=candidates,projected_lidar_points=len(points),clearance_radius_m=radius,
 limitation='Supported scan-height recesses only. Missing lidar returns are not free-space evidence. Depth evidence is local and excludes configured clipping limits; ground, low obstacles at the opening, overhead, approach path and exit connectivity unverified. No motion approval.',model_selection=None)
 a=np.asarray(grid.data).reshape(grid.info.height,grid.info.width); map_image=np.full((*a.shape,3),127,np.uint8);map_image[a==0]=255;map_image[a>=65]=0
 ix=int((origin[0]-grid.info.origin.position.x)/grid.info.resolution);iy=int((origin[1]-grid.info.origin.position.y)/grid.info.resolution);cv2.circle(map_image,(ix,iy),5,(0,0,255),-1)
 cv2.imwrite(str(args.output/'slam_map.png'),np.flipud(map_image))
 (args.output/'candidates.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
 db.close()
if __name__=='__main__':main()
