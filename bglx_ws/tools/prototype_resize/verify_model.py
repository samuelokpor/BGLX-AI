#!/usr/bin/env python3
"""Offline model consistency and sampled steering envelope verification; no motion."""
import math,sys,json,importlib.util,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parents[2]
pkg=root/'src/etrike_description';r=E.parse(pkg/'urdf/etrike.urdf.xacro').getroot()
spec=importlib.util.spec_from_file_location('geo',root/'src/bglx_navigation/bglx_navigation/concept_scan_geometry.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
assert abs(sum(float(m.get('value')) for m in r.findall('link/inertial/mass'))-40)<1e-8
for t in r.findall('link/inertial/inertia'):
 I=np.array([[float(t.get('ixx')),float(t.get('ixy')),float(t.get('ixz'))],[float(t.get('ixy')),float(t.get('iyy')),float(t.get('iyz'))],[float(t.get('ixz')),float(t.get('iyz')),float(t.get('izz'))]])
 assert np.linalg.eigvalsh(I).min()>0
 assert np.linalg.eigvalsh(np.eye(3)*np.trace(I)/2-I).min()>0
vertices=[]
for l in r.findall('link'):
 for part in l.findall('collision')+l.findall('visual'):
  geo=part.find('geometry');box=geo.find('box');cy=geo.find('cylinder');mesh=geo.find('mesh')
  if box is not None:
   sz=np.fromstring(box.get('size'),sep=' ')/2;v=np.array([[x,y,z] for x in [-sz[0],sz[0]] for y in [-sz[1],sz[1]] for z in [-sz[2],sz[2]]])
  elif cy is not None:
   rad=float(cy.get('radius'));h=float(cy.get('length'))/2;v=np.array([[rad*math.cos(t),rad*math.sin(t),z] for t in np.linspace(0,2*math.pi,361) for z in [-h,h]])
  else:
   d=(pkg/mesh.get('filename').split('package://etrike_description/')[1]).read_bytes()
   v=np.frombuffer(d,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),offset=84)['v'].reshape(-1,3)*np.fromstring(mesh.get('scale'),sep=' ')
  T=g.origin_matrix(part.find('origin'));vertices.append((l.get('name'),part.get('name'),v@T[:3,:3].T+T[:3,3]))
for a in np.linspace(-math.pi/3,math.pi/3,121):
 tf={'base_link':np.eye(4)}
 for j in r.findall('joint'):
  T=g.origin_matrix(j.find('origin'))
  if j.get('name')=='steering_joint':T[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]
  tf[j.find('child').get('link')]=tf[j.find('parent').get('link')]@T
 for name,label,v in vertices:
  T=tf[name];w=v@T[:3,:3].T+T[:3,3]
  assert w[:,0].min()>=g.REAR_X-1e-6 and w[:,0].max()<=g.FRONT_X+1e-6,(name,label,'length')
  assert np.max(abs(w[:,1]))<=g.HALF_WIDTH+1e-6,(name,label,'width')
  assert w[:,2].min()>=-1e-6,(name,label,'ground')
 assert abs(tf['front_wheel'][2,3]-.13)<1e-8
 assert abs(tf['rear_left_wheel'][2,3]-.13)<1e-8
assert abs(tf['caster_mount'][0,3]-.9)<1e-8
import yaml,ast
params=yaml.safe_load((pkg/'config/controllers.yaml').read_text())['tricycle_steering_controller']['ros__parameters']
assert params['wheelbase']==.9 and params['rear_wheels_radius']==.13 and params['front_wheels_radius']==.13
nav=yaml.safe_load((root/'src/bglx_navigation/config/nav2_params.yaml').read_text())
expected=[[-.2,-.285],[1.04,-.285],[1.04,.285],[-.2,.285]]
for name in ['local_costmap','global_costmap']:assert ast.literal_eval(nav[name][name]['ros__parameters']['footprint'])==expected
for name in ['setup','worker','forward']:
 tree=ast.parse((root/f'tools/course_supervisor_prototype/{name}.py').read_text())
 fp=next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FP' for t in n.targets))
 assert [list(p) for p in ast.literal_eval(fp)]==expected
print('PASS: 40kg, physical inertias, wheelbase/radii, footprints and sampled ±60° visual/collision envelope. No ROS or motion.')
