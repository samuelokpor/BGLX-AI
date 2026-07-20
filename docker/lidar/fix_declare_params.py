#!/usr/bin/env python3
"""Patch ydlidar_ros2_driver for ROS 2 Humble: declare_parameter needs a type."""
import re, sys
f = 'src/ydlidar_ros2_driver_node.cpp'; s = open(f).read()
T = {
 'PARAMETER_STRING':  ['port', 'frame_id', 'ignore_array'],
 'PARAMETER_INTEGER': ['baudrate', 'lidar_type', 'device_type', 'sample_rate',
                       'abnormal_check_count', 'intensity_bit', 'm1_mode', 'm2_mode', 'm3_mode'],
 'PARAMETER_BOOL':    ['resolution_fixed', 'fixed_resolution', 'reversion', 'inverted',
                       'auto_reconnect', 'isSingleChannel', 'intensity', 'support_motor_dtr',
                       'invalid_range_is_inf', 'debug'],
 'PARAMETER_DOUBLE':  ['angle_max', 'angle_min', 'range_max', 'range_min', 'frequency'],
}
m = {n: 'rclcpp::ParameterType::' + t for t, ns in T.items() for n in ns}
s = re.sub(r'declare_parameter\(\s*"([^"]+)"\s*\)',
           lambda x: f'declare_parameter("{x.group(1)}", {m[x.group(1)]})'
                     if x.group(1) in m else x.group(0), s)
left = re.findall(r'declare_parameter\(\s*"([^"]+)"\s*\)', s)
open(f, 'w').write(s)
sys.exit(1) if left else print('all declare_parameter calls patched')
