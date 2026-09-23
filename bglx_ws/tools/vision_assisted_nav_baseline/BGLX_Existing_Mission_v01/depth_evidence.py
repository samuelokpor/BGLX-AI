"""Depth evidence for the uploaded simulation rig; not motion authorization."""
import numpy as np

def decode_depth(m,maximum):
 if m.encoding!='32FC1':raise ValueError('Expected simulation 32FC1 depth in metres')
 if m.step%4:raise ValueError('Invalid float image stride')
 raw=np.frombuffer(m.data,dtype=('>f4' if m.is_bigendian else '<f4')).reshape(m.height,m.step//4)[:,:m.width]
 valid=np.isfinite(raw)&(raw>.15)&(raw<maximum-.01)
 return raw,valid

def inspect(near,transform,topics,stamp):
 reports=[]
 for topic,maximum in [('/etrike/front_camera/depth/image_raw',6.),('/etrike/front_depth/depth/image_raw',4.)]:
  if topic not in topics:
   reports.append({'topic':topic,'status':'missing'});continue
  try:
   m=near(topic,.15);info=near(topic.replace('image_raw','camera_info'),.15)
   if (m.width,m.height,m.header.frame_id)!=(info.width,info.height,info.header.frame_id):raise ValueError('Calibration mismatch')
   if any(abs(d)>1e-8 for d in info.d):raise ValueError('Rectification required')
   raw,valid=decode_depth(m,maximum);k=np.asarray(info.k).reshape(3,3)
   if k[0,0]<=0 or k[1,1]<=0:raise ValueError('Invalid camera intrinsics')
   v,u=np.indices(raw.shape);z=raw[valid]
   points=np.column_stack(((u[valid]-k[0,2])*z/k[0,0],(v[valid]-k[1,2])*z/k[1,1],z,np.ones(len(z))))
   pts=(transform('base_footprint',m.header.frame_id)@points.T).T
   r={'topic':topic,'status':'measured' if len(z) else 'no_valid_returns','configured_max_depth_m':maximum,
      'valid_pixels':int(valid.sum()),'total_pixels':int(raw.size),
      'at_or_beyond_clip_pixels':int((np.isfinite(raw)&(raw>=maximum-.01)).sum()),
      'stamp_offset_from_rgb_s':round(m.header.stamp.sec+m.header.stamp.nanosec/1e9-stamp,4)}
   if len(z):r.update(base_x_extent_m=[float(pts[:,0].min()),float(pts[:,0].max())],height_extent_m=[float(pts[:,2].min()),float(pts[:,2].max())],points_above_15cm=int((pts[:,2]>.15).sum()))
   reports.append(r)
  except (ValueError,KeyError) as e:reports.append({'topic':topic,'status':'unavailable','reason':str(e)})
 return reports
