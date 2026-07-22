#!/usr/bin/env python3
"""BGLX MPU6050 -> ROS 2 /imu publisher."""
import math, time
import smbus, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

PWR_MGMT_1 = 0x6B; ACCEL_XOUT_H = 0x3B; GYRO_XOUT_H = 0x43
G = 9.80665; ACC_LSB = 16384.0; GYR_LSB = 131.0; DEG2RAD = math.pi/180.0


class Mpu6050Imu(Node):
    def __init__(self):
        super().__init__('mpu6050_imu')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('calib_samples', 200)
        g = lambda n: self.get_parameter(n).value
        self.addr = g('i2c_addr'); self.frame = g('frame_id'); rate = float(g('rate'))
        self.bus = smbus.SMBus(int(g('i2c_bus')))
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0); time.sleep(0.1)
        n = int(g('calib_samples'))
        self.get_logger().info(f'Calibrating gyro bias ({n} samples) - keep the IMU still...')
        bx = by = bz = 0.0
        for _ in range(n):
            x, y, z = self._gyro(); bx += x; by += y; bz += z; time.sleep(0.002)
        self.bias = (bx/n, by/n, bz/n)
        self.get_logger().info(f'Gyro bias rad/s: ({self.bias[0]:+.4f}, {self.bias[1]:+.4f}, {self.bias[2]:+.4f})')
        self.pub = self.create_publisher(Imu, 'imu', 10)
        self.create_timer(1.0/rate, self.tick)
        self.get_logger().info(f'Publishing /imu at {rate:.0f} Hz (frame_id={self.frame})')

    def _rd(self, reg):
        v = (self.bus.read_byte_data(self.addr, reg) << 8) | self.bus.read_byte_data(self.addr, reg+1)
        return v - 65536 if v >= 32768 else v

    def _gyro(self):
        return (self._rd(GYRO_XOUT_H)/GYR_LSB*DEG2RAD,
                self._rd(GYRO_XOUT_H+2)/GYR_LSB*DEG2RAD,
                self._rd(GYRO_XOUT_H+4)/GYR_LSB*DEG2RAD)

    def tick(self):
        try:
            ax = self._rd(ACCEL_XOUT_H)/ACC_LSB*G
            ay = self._rd(ACCEL_XOUT_H+2)/ACC_LSB*G
            az = self._rd(ACCEL_XOUT_H+4)/ACC_LSB*G
            gx, gy, gz = self._gyro()
        except OSError:
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.linear_acceleration.x = ax; m.linear_acceleration.y = ay; m.linear_acceleration.z = az
        m.angular_velocity.x = gx - self.bias[0]
        m.angular_velocity.y = gy - self.bias[1]
        m.angular_velocity.z = gz - self.bias[2]
        m.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            m.linear_acceleration_covariance[i] = 0.01
            m.angular_velocity_covariance[i] = 0.001
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args); node = Mpu6050Imu()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
