#!/usr/bin/env python3
"""Read-only running-stack check. Does not activate nodes or publish motion."""
import math,time,json,xml.etree.ElementTree as E
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
rclpy.init();n=rclpy.create_node('prototype_stationary_check',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
def call(kind,name,req):
 c=n.create_client(kind,name)
 try:
  if not c.wait_for_service(timeout_sec=15):raise RuntimeError('Missing service '+name)
  f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=15)
  if not f.done():raise RuntimeError('Timeout '+name)
  return f.result()
 finally:n.destroy_client(c)
def param(node,key):
 q=GetParameters.Request();q.names=[key];return call(GetParameters,node+'/get_parameters',q).values[0]
try:
 for node in ['/planner_server','/controller_server','/collision_monitor']:
  state=call(GetState,node+'/get_state',GetState.Request()).current_state
  if state.id!=3:raise RuntimeError(node+' is '+state.label)
  print(node,': active')
 for node,key,want in [('/tricycle_steering_controller','wheelbase',.9),('/tricycle_steering_controller','rear_wheels_radius',.13),('/tricycle_steering_controller','front_wheels_radius',.13),('/cmd_vel_limiter','wheelbase',.9),('/cmd_vel_limiter','front_corridor_half_width',.285)]:
  v=param(node,key)
  if v.type!=3 or abs(v.double_value-want)>1e-6:raise RuntimeError(node+'/'+key+' does not match prototype')
 for node in ['/global_costmap/global_costmap','/local_costmap/local_costmap']:
  v=param(node,'footprint');fp=json.loads(v.string_value)
  if fp!=[[-.2,-.285],[1.04,-.285],[1.04,.285],[-.2,.285]]:raise RuntimeError(node+' footprint mismatch')
 desc=param('/robot_state_publisher','robot_description').string_value;r=E.fromstring(desc)
 if r.get('name')!='bglx_prototype_proxy':raise RuntimeError('Old robot_description is running')
 if abs(sum(float(m.get('value')) for m in r.findall('link/inertial/mass'))-40)>1e-6:raise RuntimeError('Robot mass mismatch')
 received={};odom=[]
 topics=['/etrike/front_scan','/etrike/left_depth/scan','/etrike/right_depth/scan','/etrike/rear_depth/scan']
 subs=[]
 for topic in topics:
  subs.append(n.create_subscription(LaserScan,topic,lambda m,t=topic:received.update({t:(time.monotonic(),m)}),qos_profile_sensor_data))
 def on_odom(m):
  v=m.twist.twist;odom.append((time.monotonic(),math.hypot(v.linear.x,v.linear.y),abs(v.angular.z)))
 subs.append(n.create_subscription(Odometry,'/tricycle_steering_controller/odometry',on_odom,qos_profile_sensor_data))
 end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 now=time.monotonic()
 for topic in topics:
  if topic not in received or now-received[topic][0]>.75:raise RuntimeError('Missing/stale scan: '+topic)
  m=received[topic][1]
  if not any(x==float('inf') or math.isfinite(x) and m.range_min<=x<=m.range_max for x in m.ranges):raise RuntimeError('Unusable scan '+topic)
 if len(odom)<5 or now-odom[-1][0]>.5 or any(v>.01 or w>.02 for t,v,w in odom if now-t<1):raise RuntimeError('Stopped odometry not confirmed')
 print('STATIONARY PASS: prototype description, controller geometry, footprint, active lifecycle nodes, fresh scans and stopped odometry.')
 print('Sensor visibility and driving performance still require the simulation regression.')
finally:
 n.destroy_node();rclpy.shutdown()
