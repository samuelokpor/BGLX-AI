#!/usr/bin/env python3

import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    return np.array([
        [
            1.0 - 2.0 * (y*y + z*z),
            2.0 * (x*y - z*w),
            2.0 * (x*z + y*w),
        ],
        [
            2.0 * (x*y + z*w),
            1.0 - 2.0 * (x*x + z*z),
            2.0 * (y*z - x*w),
        ],
        [
            2.0 * (x*z - y*w),
            2.0 * (y*z + x*w),
            1.0 - 2.0 * (x*x + y*y),
        ],
    ], dtype=np.float64)


class TerrainDetector(Node):

    def __init__(self):
        super().__init__('terrain_detector')

        # --------------------------------------------------------------
        # Topics / frames
        # --------------------------------------------------------------
        self.declare_parameter(
            'depth_topic',
            '/etrike/front_terrain/depth/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/etrike/front_terrain/depth/camera_info'
        )

        self.declare_parameter(
            'base_frame',
            'base_link'
        )

        # --------------------------------------------------------------
        # Processing rate
        # --------------------------------------------------------------
        self.declare_parameter(
            'process_hz',
            5.0
        )

        # --------------------------------------------------------------
        # Forward terrain ROI
        # --------------------------------------------------------------
        self.declare_parameter('x_min', 1.30)
        self.declare_parameter('x_max', 4.00)

        self.declare_parameter(
            'fit_half_width',
            1.25
        )

        self.declare_parameter(
            'corridor_half_width',
            0.70
        )

        # We intentionally anchor the ground fit to the near road.
        #
        # Current Phase-2 hazards begin beyond ~2.2 m, so this keeps
        # obstacle / trench geometry from corrupting the road estimate.
        self.declare_parameter(
            'ground_fit_x_max',
            2.10
        )

        # --------------------------------------------------------------
        # Positive terrain
        # --------------------------------------------------------------
        self.declare_parameter(
            'positive_threshold',
            0.035
        )

        self.declare_parameter(
            'positive_min_points',
            100
        )

        # --------------------------------------------------------------
        # Negative terrain
        # --------------------------------------------------------------
        self.declare_parameter(
            'negative_threshold',
            -0.08
        )

        self.declare_parameter(
            'negative_min_points',
            100
        )

        # --------------------------------------------------------------
        # Ground support / missing-road logic
        # --------------------------------------------------------------
        self.declare_parameter(
            'ground_support_tolerance',
            0.03
        )

        self.declare_parameter(
            'profile_bin_size',
            0.10
        )

        self.declare_parameter(
            'profile_min_points',
            20
        )

        self.declare_parameter(
            'support_min_points',
            30
        )

        self.declare_parameter(
            'missing_bins_required',
            2
        )

        self.declare_parameter(
            'missing_only_max_start',
            2.80
        )

        # --------------------------------------------------------------
        # Slope classification
        # --------------------------------------------------------------
        self.declare_parameter(
            'slope_min_grade_percent',
            3.0
        )

        self.declare_parameter(
            'slope_max_normal_grade_percent',
            12.0
        )

        self.declare_parameter(
            'slope_fit_max_rmse',
            0.005
        )

        # --------------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------------
        self.depth_topic = (
            self.get_parameter('depth_topic')
            .get_parameter_value()
            .string_value
        )

        self.info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )

        self.base_frame = (
            self.get_parameter('base_frame')
            .get_parameter_value()
            .string_value
        )

        self.process_hz = self.get_parameter(
            'process_hz'
        ).value

        self.x_min = self.get_parameter(
            'x_min'
        ).value

        self.x_max = self.get_parameter(
            'x_max'
        ).value

        self.fit_half_width = self.get_parameter(
            'fit_half_width'
        ).value

        self.corridor_half_width = self.get_parameter(
            'corridor_half_width'
        ).value

        self.ground_fit_x_max = self.get_parameter(
            'ground_fit_x_max'
        ).value

        self.positive_threshold = self.get_parameter(
            'positive_threshold'
        ).value

        self.positive_min_points = self.get_parameter(
            'positive_min_points'
        ).value

        self.negative_threshold = self.get_parameter(
            'negative_threshold'
        ).value

        self.negative_min_points = self.get_parameter(
            'negative_min_points'
        ).value

        self.ground_support_tolerance = self.get_parameter(
            'ground_support_tolerance'
        ).value

        self.profile_bin_size = self.get_parameter(
            'profile_bin_size'
        ).value

        self.profile_min_points = self.get_parameter(
            'profile_min_points'
        ).value

        self.support_min_points = self.get_parameter(
            'support_min_points'
        ).value

        self.missing_bins_required = self.get_parameter(
            'missing_bins_required'
        ).value

        self.missing_only_max_start = self.get_parameter(
            'missing_only_max_start'
        ).value

        self.slope_min_grade_percent = self.get_parameter(
            'slope_min_grade_percent'
        ).value

        self.slope_max_normal_grade_percent = self.get_parameter(
            'slope_max_normal_grade_percent'
        ).value

        self.slope_fit_max_rmse = self.get_parameter(
            'slope_fit_max_rmse'
        ).value

        # --------------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------------
        self.latest_depth = None
        self.latest_frame = None

        self.camera_info = None

        self.image_sequence = 0
        self.processed_sequence = -1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # --------------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------------
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.info_callback,
            qos_profile_sensor_data
        )

        self.hazard_pub = self.create_publisher(
            Bool,
            '/etrike/terrain/hazard',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/etrike/terrain/status',
            10
        )

        period = 1.0 / max(
            0.5,
            float(self.process_hz)
        )

        self.timer = self.create_timer(
            period,
            self.process
        )

        self.last_state = None
        self.last_tf_warning = 0.0

        self.get_logger().info(
            'Terrain detector started '
            '(diagnostic only; no motion authority)'
        )

        self.get_logger().info(
            f'Depth: {self.depth_topic}'
        )

    # ------------------------------------------------------------------
    # Input
    # ------------------------------------------------------------------

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):

        if msg.encoding.upper() != '32FC1':
            self.get_logger().warning(
                'Expected 32FC1 depth image, '
                f'got {msg.encoding}',
                throttle_duration_sec=5.0
            )
            return

        try:
            raw = np.frombuffer(
                msg.data,
                dtype='<f4'
            ).reshape(
                msg.height,
                msg.step // 4
            )

        except Exception as exc:
            self.get_logger().error(
                f'Unable to decode depth image: {exc}'
            )
            return

        depth = raw[:, :msg.width].copy()

        depth[
            (~np.isfinite(depth)) |
            (depth < 0.15) |
            (depth > 4.0)
        ] = np.nan

        self.latest_depth = depth
        self.latest_frame = msg.header.frame_id
        self.image_sequence += 1

    # ------------------------------------------------------------------
    # Ground-plane fit
    # ------------------------------------------------------------------

    @staticmethod
    def fit_ground_plane(x, y, z):

        if len(z) < 300:
            return None

        gx = x.copy()
        gy = y.copy()
        gz = z.copy()

        coeff = None

        for _ in range(5):

            if len(gz) < 300:
                return None

            A = np.column_stack([
                gx,
                gy,
                np.ones_like(gx)
            ])

            coeff, _, _, _ = np.linalg.lstsq(
                A,
                gz,
                rcond=None
            )

            predicted = (
                coeff[0] * gx +
                coeff[1] * gy +
                coeff[2]
            )

            residual = gz - predicted

            keep = (
                np.abs(residual) < 0.015
            )

            gx = gx[keep]
            gy = gy[keep]
            gz = gz[keep]

        if coeff is None:
            return None

        return (
            float(coeff[0]),
            float(coeff[1]),
            float(coeff[2]),
            int(len(gz))
        )

    # ------------------------------------------------------------------
    # Longitudinal profile
    # ------------------------------------------------------------------

    def build_profile(
        self,
        x,
        relative_height,
        corridor_mask
    ):

        half_window = (
            self.profile_bin_size * 0.40
        )

        centers = np.arange(
            max(1.50, self.x_min),
            min(3.60, self.x_max) + 1e-6,
            self.profile_bin_size
        )

        result = []

        for xc in centers:

            mask = (
                corridor_mask &
                (x >= xc - half_window) &
                (x < xc + half_window)
            )

            hh = relative_height[mask]

            hh = hh[
                np.isfinite(hh) &
                (hh > -0.50) &
                (hh < 0.40)
            ]

            n_total = int(len(hh))

            if n_total:
                median = float(
                    np.median(hh)
                )

                ground_points = int(
                    np.count_nonzero(
                        np.abs(hh) <
                        self.ground_support_tolerance
                    )
                )

                deep_points = int(
                    np.count_nonzero(
                        hh <
                        self.negative_threshold
                    )
                )

            else:
                median = None
                ground_points = 0
                deep_points = 0

            result.append({
                'x': float(xc),
                'n': n_total,
                'median': median,
                'ground': ground_points,
                'deep': deep_points,
            })

        return result

    # ------------------------------------------------------------------
    # Missing-ground reasoning
    # ------------------------------------------------------------------

    def detect_support_gap(self, profile):

        support = []

        for p in profile:
            support.append(
                p['n'] >= self.profile_min_points and
                p['ground'] >= self.support_min_points
            )

        deep = []

        for p in profile:
            deep.append(
                p['deep'] >= self.support_min_points
            )

        missing = []

        for p in profile:
            missing.append(
                p['n'] < self.profile_min_points
            )

        # --------------------------------------------------------------
        # Strong case:
        #
        # normal road -> missing expected ground -> deeper return
        # --------------------------------------------------------------

        for deep_index, is_deep in enumerate(deep):

            if not is_deep:
                continue

            prior_support = [
                i for i in range(deep_index)
                if support[i]
            ]

            if not prior_support:
                continue

            last_support = prior_support[-1]

            gap_count = sum(
                1
                for i in range(
                    last_support + 1,
                    deep_index
                )
                if missing[i]
            )

            if gap_count >= self.missing_bins_required:

                return {
                    'detected': True,
                    'deep_confirmed': True,
                    'edge_m': profile[
                        last_support
                    ]['x'],
                    'missing_bins': gap_count,
                }

        # --------------------------------------------------------------
        # Missing-only candidate.
        #
        # Keep this conservative because image horizon / occlusion may
        # also remove ground returns.
        # --------------------------------------------------------------

        run_start = None

        for i, is_missing in enumerate(missing):

            if is_missing:

                if run_start is None:
                    run_start = i

            else:

                if run_start is not None:

                    run_length = i - run_start

                    if (
                        run_length >= self.missing_bins_required and
                        run_start > 0 and
                        support[run_start - 1] and
                        profile[run_start]['x']
                        <= self.missing_only_max_start
                    ):

                        return {
                            'detected': True,
                            'deep_confirmed': False,
                            'edge_m': profile[
                                run_start - 1
                            ]['x'],
                            'missing_bins': run_length,
                        }

                    run_start = None

        return {
            'detected': False,
            'deep_confirmed': False,
            'edge_m': None,
            'missing_bins': 0,
        }

    # ------------------------------------------------------------------
    # Ramp / slope fit
    # ------------------------------------------------------------------

    def fit_positive_slope(self, profile):

        xs = []
        hs = []

        for p in profile:

            if (
                p['n'] >= self.profile_min_points and
                p['median'] is not None and
                p['median'] > 0.005 and
                p['median'] < 0.20
            ):
                xs.append(p['x'])
                hs.append(p['median'])

        if len(xs) < 5:
            return None

        xs = np.asarray(
            xs,
            dtype=np.float64
        )

        hs = np.asarray(
            hs,
            dtype=np.float64
        )

        # Find longest roughly contiguous run.
        split_locations = np.where(
            np.diff(xs) >
            self.profile_bin_size * 1.6
        )[0]

        groups = np.split(
            np.arange(len(xs)),
            split_locations + 1
        )

        group = max(
            groups,
            key=len
        )

        if len(group) < 5:
            return None

        xs = xs[group]
        hs = hs[group]

        A = np.column_stack([
            xs,
            np.ones_like(xs)
        ])

        fit, _, _, _ = np.linalg.lstsq(
            A,
            hs,
            rcond=None
        )

        slope = float(fit[0])
        intercept = float(fit[1])

        predicted = (
            slope * xs +
            intercept
        )

        rmse = float(
            np.sqrt(
                np.mean(
                    (hs - predicted) ** 2
                )
            )
        )

        grade_percent = (
            100.0 * slope
        )

        angle_deg = math.degrees(
            math.atan(slope)
        )

        return {
            'slope': slope,
            'grade_percent': grade_percent,
            'angle_deg': angle_deg,
            'rmse': rmse,
            'bins': int(len(xs)),
        }

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def publish_status(
        self,
        state,
        hazard,
        details
    ):

        hazard_msg = Bool()
        hazard_msg.data = bool(hazard)

        self.hazard_pub.publish(
            hazard_msg
        )

        payload = {
            'state': state,
            'hazard': bool(hazard),
            **details
        }

        status_msg = String()
        status_msg.data = json.dumps(
            payload,
            separators=(',', ':')
        )

        self.status_pub.publish(
            status_msg
        )

        if state != self.last_state:

            self.get_logger().info(
                f'TERRAIN STATE: '
                f'{self.last_state} -> {state}'
            )

            self.last_state = state

    # ------------------------------------------------------------------
    # Main processing
    # ------------------------------------------------------------------

    def process(self):

        if self.latest_depth is None:
            return

        if self.camera_info is None:
            return

        if self.image_sequence == self.processed_sequence:
            return

        self.processed_sequence = self.image_sequence

        depth = self.latest_depth
        info = self.camera_info

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        source_frame = (
            self.latest_frame
            if self.latest_frame
            else 'depth_camera_optical'
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rclpy.time.Time()
            )

        except Exception as exc:

            now = time.monotonic()

            if now - self.last_tf_warning > 5.0:
                self.get_logger().warning(
                    f'TF {source_frame} -> '
                    f'{self.base_frame} unavailable: '
                    f'{exc}'
                )

                self.last_tf_warning = now

            return

        R = quat_to_matrix(
            tf.transform.rotation
        )

        T = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z
        ], dtype=np.float64)

        h, w = depth.shape

        v, u = np.indices(
            (h, w)
        )

        Z = depth

        valid = np.isfinite(Z)

        if np.count_nonzero(valid) < 500:
            return

        X = (
            (u - cx) *
            Z / fx
        )

        Y = (
            (v - cy) *
            Z / fy
        )

        P = np.stack([
            X[valid],
            Y[valid],
            Z[valid]
        ], axis=0)

        B = (
            R @ P
        ) + T[:, None]

        x = B[0]
        y = B[1]
        z = B[2]

        roi = (
            (x > self.x_min) &
            (x < self.x_max) &
            (np.abs(y) < self.fit_half_width) &
            (z > -0.50) &
            (z < 0.50)
        )

        x = x[roi]
        y = y[roi]
        z = z[roi]

        if len(z) < 500:
            return

        # --------------------------------------------------------------
        # Near-field ground anchor.
        # --------------------------------------------------------------

        ground_candidate = (
            (x <= self.ground_fit_x_max) &
            (z > -0.25) &
            (z < 0.20)
        )

        fit = self.fit_ground_plane(
            x[ground_candidate],
            y[ground_candidate],
            z[ground_candidate]
        )

        if fit is None:

            self.publish_status(
                'GROUND_FIT_FAILED',
                True,
                {
                    'roi_points': int(len(z))
                }
            )

            return

        a, b, c, ground_points = fit

        expected_ground = (
            a*x +
            b*y +
            c
        )

        relative_height = (
            z -
            expected_ground
        )

        corridor = (
            np.abs(y) <
            self.corridor_half_width
        )

        positive = (
            corridor &
            (relative_height >
             self.positive_threshold)
        )

        negative = (
            corridor &
            (relative_height <
             self.negative_threshold)
        )

        positive_count = int(
            np.count_nonzero(positive)
        )

        negative_count = int(
            np.count_nonzero(negative)
        )

        positive_height_cm = None

        if positive_count:

            positive_height_cm = float(
                np.percentile(
                    relative_height[positive],
                    95
                ) * 100.0
            )

        negative_depth_cm = None

        if negative_count:

            negative_depth_cm = float(
                -np.median(
                    relative_height[negative]
                ) * 100.0
            )

        profile = self.build_profile(
            x,
            relative_height,
            corridor
        )

        support_gap = (
            self.detect_support_gap(
                profile
            )
        )

        slope_fit = (
            self.fit_positive_slope(
                profile
            )
        )

        # The local ground plane itself may also be inclined if the
        # vehicle is already on a slope.
        local_grade_percent = (
            100.0 * a
        )

        local_angle_deg = math.degrees(
            math.atan(a)
        )

        # --------------------------------------------------------------
        # Classification priority:
        #
        # 1. negative terrain
        # 2. continuous slope
        # 3. positive obstacle
        # 4. clear
        # --------------------------------------------------------------

        negative_detected = (
            negative_count >=
            self.negative_min_points
        )

        if (
            negative_detected or
            support_gap['detected']
        ):

            state = 'NEGATIVE_TERRAIN'
            hazard = True

        else:

            slope_detected = False
            slope_grade = None
            slope_angle = None
            slope_rmse_cm = None

            if slope_fit is not None:

                abs_grade = abs(
                    slope_fit[
                        'grade_percent'
                    ]
                )

                if (
                    abs_grade >=
                    self.slope_min_grade_percent and
                    slope_fit['rmse'] <=
                    self.slope_fit_max_rmse
                ):
                    slope_detected = True

                    slope_grade = slope_fit[
                        'grade_percent'
                    ]

                    slope_angle = slope_fit[
                        'angle_deg'
                    ]

                    slope_rmse_cm = (
                        slope_fit['rmse'] *
                        100.0
                    )

            if (
                not slope_detected and
                abs(local_grade_percent) >=
                self.slope_min_grade_percent
            ):
                slope_detected = True
                slope_grade = (
                    local_grade_percent
                )
                slope_angle = (
                    local_angle_deg
                )
                slope_rmse_cm = 0.0

            if slope_detected:

                if (
                    abs(slope_grade) >
                    self.slope_max_normal_grade_percent
                ):
                    state = 'STEEP_SLOPE'
                    hazard = True

                else:
                    state = 'SLOPE'
                    hazard = False

            elif (
                positive_count >=
                self.positive_min_points
            ):

                state = 'POSITIVE_OBSTACLE'
                hazard = True

            else:

                state = 'CLEAR'
                hazard = False

        details = {
            'positive_points':
                positive_count,

            'positive_height_cm':
                positive_height_cm,

            'negative_points':
                negative_count,

            'negative_depth_cm':
                negative_depth_cm,

            'missing_ground':
                bool(
                    support_gap[
                        'detected'
                    ]
                ),

            'deep_return_confirmed':
                bool(
                    support_gap[
                        'deep_confirmed'
                    ]
                ),

            'estimated_drop_edge_m':
                support_gap[
                    'edge_m'
                ],

            'missing_bins':
                int(
                    support_gap[
                        'missing_bins'
                    ]
                ),

            'ground_plane_a':
                round(a, 6),

            'ground_plane_b':
                round(b, 6),

            'ground_plane_c':
                round(c, 6),

            'local_ground_grade_percent':
                round(
                    local_grade_percent,
                    3
                ),

            'ground_points':
                ground_points,

            'roi_points':
                int(len(z)),
        }

        if slope_fit is not None:

            details[
                'profile_slope_grade_percent'
            ] = round(
                slope_fit[
                    'grade_percent'
                ],
                3
            )

            details[
                'profile_slope_angle_deg'
            ] = round(
                slope_fit[
                    'angle_deg'
                ],
                3
            )

            details[
                'profile_slope_rmse_cm'
            ] = round(
                slope_fit[
                    'rmse'
                ] * 100.0,
                3
            )

        self.publish_status(
            state,
            hazard,
            details
        )


def main(args=None):

    rclpy.init(args=args)

    node = TerrainDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
