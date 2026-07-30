#!/usr/bin/env python3
"""Generate an Oxford-college-style Gazebo world for the BGLX e-trike.

Not an architectural replica. A geometric one: the layout properties that
make an Oxford college hard and interesting to navigate.

  - Two enclosed quadrangles, so the map is bounded and stops growing
  - Gate archways at 3.5m, near the limit of a 1.33m-wheelbase trike
  - A cloister of repeated columns: the pathological case for scan matching
  - A service lane and loading bay, where a delivery vehicle actually goes
  - Bollards, benches, trees and bike racks as small obstacles
  - A perimeter wall enclosing everything

Coordinates: origin at the centre of Front Quad, +x east, +y north.

    python3 gen_oxford.py > oxford_college.world
"""

import math
import sys

# --- dimensions (metres) --------------------------------------------------
FRONT_QUAD = 15.0          # half-width of the enclosed lawn
SECOND_QUAD = 12.0
WING_DEPTH = 8.0           # building thickness
WING_HEIGHT = 12.0
GATE_WIDTH = 3.5           # the tight one, deliberately
PASSAGE_WIDTH = 4.5
LINK_LENGTH = 14.0         # cloister run between the two quads
WALL_HEIGHT = 3.0
WALL_THICK = 0.5

SECOND_QUAD_Y = (FRONT_QUAD + WING_DEPTH) + LINK_LENGTH + (SECOND_QUAD + WING_DEPTH)

parts = []


def box(name, x, y, z, sx, sy, sz, yaw=0.0, colour="Gazebo/Grey",
        static=True):
    parts.append("""
    <model name="{n}">
      <static>{st}</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <surface>
            <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{c}</name>
          </script></material>
        </visual>
      </link>
    </model>""".format(n=name, st="true" if static else "false", x=x, y=y, z=z,
                       sx=sx, sy=sy, sz=sz, yaw=yaw, c=colour))


def cyl(name, x, y, z, r, h, colour="Gazebo/Grey"):
    parts.append("""
    <model name="{n}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>{c}</name>
          </script></material>
        </visual>
      </link>
    </model>""".format(n=name, x=x, y=y, z=z, r=r, h=h, c=colour))


def wing(tag, cx, cy, length, depth, height, horizontal, gap=0.0,
         colour="Gazebo/Residential"):
    """A range of building. If gap > 0, split it to leave an archway."""
    if gap <= 0.0:
        if horizontal:
            box(tag, cx, cy, height / 2, length, depth, height, colour=colour)
        else:
            box(tag, cx, cy, height / 2, depth, length, height, colour=colour)
        return

    seg = (length - gap) / 2.0
    off = gap / 2.0 + seg / 2.0
    if horizontal:
        box(tag + "_a", cx - off, cy, height / 2, seg, depth, height, colour=colour)
        box(tag + "_b", cx + off, cy, height / 2, seg, depth, height, colour=colour)
        box(tag + "_lintel", cx, cy, height / 2 + 2.6, gap, depth,
            height - 5.2, colour=colour)
    else:
        box(tag + "_a", cx, cy - off, height / 2, depth, seg, height, colour=colour)
        box(tag + "_b", cx, cy + off, height / 2, depth, seg, height, colour=colour)
        box(tag + "_lintel", cx, cy, height / 2 + 2.6, depth, gap,
            height - 5.2, colour=colour)


def quad(tag, cy, half, gap_south, gap_north, colour="Gazebo/Residential"):
    """Four ranges enclosing a lawn, with optional archways N and S."""
    outer = half + WING_DEPTH
    span = 2 * outer
    wing(tag + "_south", 0.0, cy - (half + WING_DEPTH / 2), span, WING_DEPTH,
         WING_HEIGHT, True, gap_south, colour)
    wing(tag + "_north", 0.0, cy + (half + WING_DEPTH / 2), span, WING_DEPTH,
         WING_HEIGHT, True, gap_north, colour)
    wing(tag + "_west", -(half + WING_DEPTH / 2), cy, 2 * half, WING_DEPTH,
         WING_HEIGHT, False, 0.0, colour)
    wing(tag + "_east", (half + WING_DEPTH / 2), cy, 2 * half, WING_DEPTH,
         WING_HEIGHT, False, 0.0, colour)
    box(tag + "_lawn", 0.0, cy, 0.005, 2 * half - 4.0, 2 * half - 4.0, 0.01,
        colour="Gazebo/Green")


# --- the college ----------------------------------------------------------
quad("front", 0.0, FRONT_QUAD, GATE_WIDTH, PASSAGE_WIDTH)
quad("second", SECOND_QUAD_Y, SECOND_QUAD, PASSAGE_WIDTH, 0.0,
     colour="Gazebo/Wood")

link_y0 = FRONT_QUAD + WING_DEPTH
link_y1 = link_y0 + LINK_LENGTH
half_link = PASSAGE_WIDTH / 2 + 2.0
box("cloister_wall_w", -half_link, (link_y0 + link_y1) / 2, WALL_HEIGHT / 2,
    WALL_THICK, LINK_LENGTH, WALL_HEIGHT)
n_cols = 6
for i in range(n_cols):
    y = link_y0 + (i + 0.5) * LINK_LENGTH / n_cols
    cyl("cloister_col_%d" % i, half_link, y, 2.0, 0.35, 4.0,
        colour="Gazebo/White")
box("cloister_roof", 0.0, (link_y0 + link_y1) / 2, 4.2,
    2 * half_link, LINK_LENGTH, 0.4, colour="Gazebo/Wood")

lane_x = FRONT_QUAD + WING_DEPTH + 7.0
box("service_wall_e", lane_x + 7.0, SECOND_QUAD_Y / 2, WALL_HEIGHT / 2,
    WALL_THICK, SECOND_QUAD_Y + 40.0, WALL_HEIGHT)
box("loading_bay", lane_x + 3.0, 6.0, 1.6, 6.0, 10.0, 3.2,
    colour="Gazebo/DarkGrey")
box("bin_store", lane_x + 3.0, 22.0, 1.0, 4.0, 5.0, 2.0,
    colour="Gazebo/DarkGrey")

box("porters_lodge", -7.0, -(FRONT_QUAD + WING_DEPTH + 4.0), 1.6,
    5.0, 6.0, 3.2, colour="Gazebo/Wood")

for i, gx in enumerate((-4.0, -2.4, 2.4, 4.0)):
    cyl("bollard_gate_%d" % i, gx, -(FRONT_QUAD + WING_DEPTH + 8.0), 0.5,
        0.12, 1.0, colour="Gazebo/Black")

for i, (bx, by) in enumerate(((-11.0, -11.0), (11.0, -11.0))):
    box("bike_rack_%d" % i, bx, by, 0.45, 3.0, 0.6, 0.9,
        colour="Gazebo/Grey")

for i, (tx, ty) in enumerate(((-6.0, -6.0), (6.0, -6.0), (-6.0, 6.0),
                              (6.0, 6.0))):
    cyl("tree_trunk_%d" % i, tx, SECOND_QUAD_Y + ty, 1.5, 0.22, 3.0,
        colour="Gazebo/Wood")
    cyl("tree_crown_%d" % i, tx, SECOND_QUAD_Y + ty, 4.0, 2.2, 2.5,
        colour="Gazebo/Green")

for i, (bx, by, yaw) in enumerate(((0.0, -12.0, 0.0), (0.0, 12.0, 0.0),
                                   (-12.0, 0.0, math.pi / 2),
                                   (12.0, 0.0, math.pi / 2))):
    box("bench_%d" % i, bx, by, 0.3, 1.8, 0.5, 0.6, yaw=yaw,
        colour="Gazebo/Wood")

per_x0, per_x1 = -(FRONT_QUAD + WING_DEPTH + 14.0), lane_x + 14.0
per_y0, per_y1 = -(FRONT_QUAD + WING_DEPTH + 22.0), SECOND_QUAD_Y + SECOND_QUAD + WING_DEPTH + 14.0
cx, cy = (per_x0 + per_x1) / 2, (per_y0 + per_y1) / 2
w, h = per_x1 - per_x0, per_y1 - per_y0
box("perim_s", cx, per_y0, WALL_HEIGHT / 2, w, WALL_THICK, WALL_HEIGHT)
box("perim_n", cx, per_y1, WALL_HEIGHT / 2, w, WALL_THICK, WALL_HEIGHT)
box("perim_w", per_x0, cy, WALL_HEIGHT / 2, WALL_THICK, h, WALL_HEIGHT)
box("perim_e", per_x1, cy, WALL_HEIGHT / 2, WALL_THICK, h, WALL_HEIGHT)

world = """<?xml version="1.0" ?>
<!-- Oxford-college-style world for the BGLX autonomous delivery e-trike.
     Generated by gen_oxford.py - edit the generator, not this file.
-->
<sdf version="1.6">
  <world name="oxford_college">

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
%s

  </world>
</sdf>
""" % "".join(parts)

sys.stdout.write(world)
