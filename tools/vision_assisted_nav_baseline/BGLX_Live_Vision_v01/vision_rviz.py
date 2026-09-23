#!/usr/bin/env python3
"""RViz annotations from mission logs; publishes visual markers only."""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

PURPLE = (0.71, 0.24, 1.0)
GREEN = (0.25, 1.0, 0.5)
AMBER = (1.0, 0.7, 0.15)
RED = (1.0, 0.25, 0.25)
STATES = {
    'configuration': ('NAV2 CRUISE | scanning for openings', GREEN),
    'inspection_trigger': ('OPENING DETECTED | requesting inspection', AMBER),
    'handoff_cancel_requested': ('HANDOFF | cancelling Nav2', AMBER),
    'handoff_stopped': ('STOP CONFIRMED | preparing inspection', AMBER),
    'view_plan': ('VIEWING MANOEUVRE | planned path in purple', PURPLE),
    'opening_in_view': ('OPENING IN VIEW | preparing capture', PURPLE),
    'vision_capture': ('VISION | capture + Qwen ranking', PURPLE),
    'vision_ranking': ('VISION | checking ranked opening', PURPLE),
    'vision_selected': ('VISION ASSIST | backup / align / cross', PURPLE),
    'vision_crossing_complete': ('CROSSING CONFIRMED | resuming Nav2 to C', GREEN),
    'abort': ('TRIAL ABORTED | awaiting stop confirmation', RED),
    'plan_only': ('PLAN ONLY | no driving requested', AMBER),
    'vision_plan_only_complete': ('PLAN ONLY | no driving requested', AMBER),
}

class VisionDisplay(Node):
    def __init__(self):
        super().__init__('bglx_vision_display')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            MarkerArray, '/bglx/vision/status_markers', qos)
        self.root = Path.home() / 'bglx_navtest/logs'
        self.file, self.offset = None, 0
        self.reset()
        self.create_timer(0.5, self.tick)
        print('Visual display running; no movement commands.', flush=True)

    def reset(self):
        self.label, self.color = 'Waiting for next live-vision run', AMBER
        self.gate, self.gate_label = None, ''
        self.frame, self.finished = 'odom', False

    def consume(self, row):
        event = row.get('event', '')
        if event in STATES:
            self.label, self.color = STATES[event]
        if event == 'inspection_trigger':
            self.frame = row['frame']
            self.gate = row['target']['mouth'][:2]
            self.gate_label = 'Lidar inspection target'
        if event == 'stationary_openings':
            self.frame = row['frame']
        if event in ('view_plan', 'opening_in_view'):
            self.gate = row['target']['mouth'][:2]
            self.gate_label = 'Viewing target'
        if event == 'vision_selected':
            self.frame, self.gate = row['frame'], row['gate'][:2]
            self.gate_label = row['candidate_id'] + ' | selected + geometry checked'
        if event == 'vision_crossing_complete':
            self.gate_label = 'Vision crossing confirmed'
        if event == 'finish':
            self.finished = True
            ok = row.get('success', False)
            if not row.get('stop_confirmed', False):
                self.label = 'TRIAL ENDED | stop NOT confirmed'
            elif not row.get('execute', False):
                self.label = 'PLAN ONLY FINISHED | robot stopped'
            elif ok and row.get('vision_crossing_confirmed', False):
                self.label = 'ARRIVED C | vision crossing + stop confirmed'
            else:
                self.label = 'TRIAL ENDED | robot stopped; see log'
            self.color = GREEN if ok and row.get('stop_confirmed') else RED

    def marker(self, ident, frame, kind, xyz, color):
        m = Marker()
        m.header.frame_id = frame
        m.ns, m.id = 'bglx_vision_ui', ident
        m.type, m.action = kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
        m.frame_locked = True
        return m

    def publish_status(self):
        status = self.marker(
            0, 'base_footprint', Marker.TEXT_VIEW_FACING,
            (0., 0., 2.7), self.color)
        status.scale.z = 0.38
        status.text = 'LAST MISSION EVENT\n' + self.label
        if self.file:
            status.text += '\n' + self.file.parent.name
        markers = [status]
        if self.gate is not None and not self.finished:
            x, y = self.gate
            dot = self.marker(
                1, self.frame, Marker.CYLINDER, (x, y, 0.05), PURPLE)
            dot.scale.x, dot.scale.y, dot.scale.z = 0.65, 0.65, 0.08
            label = self.marker(
                2, self.frame, Marker.TEXT_VIEW_FACING, (x, y, 1.7), PURPLE)
            label.text, label.scale.z = self.gate_label, 0.35
            markers.extend([dot, label])
        else:
            for ident in (1, 2):
                delete = self.marker(
                    ident, 'base_footprint', Marker.CUBE, (0., 0., 0.), PURPLE)
                delete.action = Marker.DELETE
                markers.append(delete)
        self.pub.publish(MarkerArray(markers=markers))

    def tick(self):
        files = sorted(self.root.glob('live_vision_*/events.jsonl'))
        if files:
            newest = files[-1]
            if newest != self.file:
                self.file, self.offset = newest, 0
                self.reset()
                print('Following:', newest, flush=True)
            try:
                with self.file.open('rb') as stream:
                    stream.seek(self.offset)
                    chunk = stream.read(65536)
                complete = chunk.rfind(b'\n') + 1
                self.offset += complete
                for line in chunk[:complete].splitlines():
                    self.consume(json.loads(line))
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.label = 'DISPLAY READ ERROR | ' + str(error)[:70]
                self.color = RED
        self.publish_status()

if __name__ == '__main__':
    rclpy.init()
    node = VisionDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
