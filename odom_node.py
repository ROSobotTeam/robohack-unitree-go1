#!/usr/bin/env python3
"""T4: одометрия Go1 (v2: откалибровано по логам симулятора).

Курс (yaw) берётся из ориентации /imu/data, скорость - из /cmd_vel.
Публикует /odom (odom -> trunk) и TF odom -> trunk с частотой 50 Гц по sim time.

Калибровка (по записям ground truth в Gazebo/CHAMP, 4 прямых участка):
  - реальная скорость вперёд ~0.917 от команды -> speed_scale = 0.917;
  - при повороте на месте корпус смещается назад ~0.013 м/с -> turn_vx = -0.01;
  - при ходьбе вперёд есть боковой снос влево ~7% от скорости -> lateral_ratio = 0.07.
Ограничение: скорость берётся из /cmd_vel, поэтому, если робот упёрся в препятствие,
а команды продолжают идти, одометрия «проедет» дальше, чем робот на самом деле.
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomNode(Node):
    def __init__(self):
        super().__init__('go1_odom', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('speed_scale', 0.917)  # реальная скорость / команда
        self.declare_parameter('turn_vx', -0.01)      # м/с, смещение при повороте на месте
        self.declare_parameter('lateral_ratio', 0.07) # боковой снос влево / скорость вперёд
        self.declare_parameter('cmd_timeout', 0.5)   # с: нет команд -> скорость 0

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.turn_vx = float(self.get_parameter('turn_vx').value)
        self.lateral_ratio = float(self.get_parameter('lateral_ratio').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        rate = float(self.get_parameter('rate_hz').value)

        # Состояние одометрии (поза относительно старта)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw0 = None        # курс IMU в момент первого сообщения
        self.wz = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.cmd_wz = 0.0
        self.cmd_t = -1e9
        self.last_t = None

        # best_effort подходит и под reliable, и под best_effort издателей
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, qos_profile_sensor_data)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = TransformBroadcaster(self)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('go1_odom запущен (use_sim_time=true)')

    def imu_cb(self, msg):
        q = msg.orientation
        vals = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            return
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 0.5:
            return
        raw = yaw_from_quat(q)
        if self.yaw0 is None:
            self.yaw0 = raw
        self.yaw = wrap(raw - self.yaw0)
        if math.isfinite(msg.angular_velocity.z):
            self.wz = msg.angular_velocity.z

    def cmd_cb(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.linear.y):
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.cmd_wz = msg.angular.z if math.isfinite(msg.angular.z) else 0.0
            self.cmd_t = self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        dt = 0.0
        if self.last_t is not None:
            dt = t - self.last_t
            if dt < 0.0 or dt > 1.0:
                dt = 0.0
        self.last_t = t

        vx, vy, wz = self.vx, self.vy, self.cmd_wz
        if t - self.cmd_t > self.cmd_timeout:
            vx = vy = wz = 0.0
        in_place_turn = abs(wz) > 0.05 and abs(vx) < 0.01 and abs(vy) < 0.01
        fwd_cmd = max(vx, 0.0)
        vx *= self.speed_scale
        vy *= self.speed_scale
        vy += self.lateral_ratio * fwd_cmd
        if in_place_turn:
            vx += self.turn_vx

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'trunk'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.02
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'trunk'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomNode()
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
