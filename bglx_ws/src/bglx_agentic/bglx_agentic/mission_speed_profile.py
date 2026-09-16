"""Apply the Delivery C simulation profile, without sending a motion goal."""
import math
import rclpy
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter


def main():
    rclpy.init()
    node = rclpy.create_node('bglx_delivery_speed_profile')

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError('unavailable: '+name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
            if not future.done() or future.result() is None:
                raise RuntimeError('no response: '+name)
            return future.result()
        finally:
            node.destroy_client(client)

    def get(server, names):
        values = call(GetParameters, server+'/get_parameters', GetParameters.Request(names=names)).values
        if len(values) != len(names):
            raise RuntimeError('incomplete parameter response: '+server)
        def value(v):
            if v.type == 1: return v.bool_value
            if v.type == 2: return v.integer_value
            if v.type == 3: return v.double_value
            if v.type == 4: return v.string_value
            raise RuntimeError('missing/unsupported parameter: '+server)
        return dict(zip(names, map(value, values)))

    def apply(server, values):
        params = [Parameter(name, value=value).to_parameter_msg() for name, value in values.items()]
        result = call(SetParametersAtomically, server+'/set_parameters_atomically',
                      SetParametersAtomically.Request(parameters=params)).result
        if not result.successful:
            raise RuntimeError(server+' refused settings: '+result.reason)
        actual = get(server, list(values))
        if actual != values:
            raise RuntimeError('settings did not read back correctly: '+server)

    try:
        controller = get('/controller_server', ['use_sim_time', 'FollowPath.plugin',
                         'FollowPath.use_collision_detection', 'FollowPath.allow_reversing',
                         'FollowPath.use_rotate_to_heading'])
        if (controller['use_sim_time'] is not True
                or 'RegulatedPurePursuitController' not in controller['FollowPath.plugin']
                or controller['FollowPath.use_collision_detection'] is not True
                or controller['FollowPath.allow_reversing'] is not False
                or controller['FollowPath.use_rotate_to_heading'] is not False):
            raise RuntimeError('expected simulated, forward-only RPP with collision detection')
        local = get('/local_costmap/local_costmap', ['inflation_layer.enabled',
                    'inflation_layer.cost_scaling_factor', 'inflation_layer.inflation_radius'])
        factor = local['inflation_layer.cost_scaling_factor']
        if (local['inflation_layer.enabled'] is not True or not math.isfinite(factor) or factor <= 0
                or local['inflation_layer.inflation_radius'] < 1.2):
            raise RuntimeError('proximity slowdown needs enabled inflation with radius >=1.2m')
        limits = get('/cmd_vel_limiter', ['use_speed_scaled_cap', 'max_linear_vel',
                     'fail_closed_on_front_scan_loss', 'fail_closed_on_rear_scan_loss',
                     'fail_closed_on_left_scan_loss', 'fail_closed_on_right_scan_loss',
                     'fail_closed_on_terrain_loss'])
        if limits['max_linear_vel'] < .95 or any(v is not True for k,v in limits.items() if k != 'max_linear_vel'):
            raise RuntimeError('expected speed-scaled limiter and all sensor-loss stop protections')
        profile = {'FollowPath.desired_linear_vel': .95,
                   'FollowPath.use_regulated_linear_velocity_scaling': True,
                   'FollowPath.use_cost_regulated_linear_velocity_scaling': True,
                   'FollowPath.cost_scaling_dist': 1.2,
                   'FollowPath.cost_scaling_gain': 1.,
                   'FollowPath.inflation_cost_scaling_factor': float(factor),
                   'FollowPath.regulated_linear_scaling_min_radius': 2.,
                   'FollowPath.regulated_linear_scaling_min_speed': .25}
        apply('/planner_server', {'GridBased.change_penalty': 0.})
        apply('/controller_server', profile)
        print('PROFILE READY: cruise <=0.95m/s; curvature + proximity slowdown; alignment <=0.25m/s.', flush=True)
        print('Reverse assistance requests 0.40m/s only after the extended swept-area audit.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
