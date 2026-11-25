import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowCmd


SPEED_MPS = 0.3
WARMUP_S = 1.5
LIMIT_S = 90.0


def pose_sample(msg):
    stamp = msg.header.stamp
    pos = msg.pose.pose.position
    o = msg.pose.pose.orientation
    roll = math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x**2 + o.y**2))
    sin_pitch = 2 * (o.w * o.y - o.z * o.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y**2 + o.z**2))
    return stamp.sec + stamp.nanosec / 1e9, pos, roll, pitch, yaw


class WalkingCheck(Node):
    def __init__(self, speed_mps, limit_s):
        super().__init__("local_t3_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self._on_pose, qos_profile_sensor_data)
        self.create_subscription(LowCmd, "/lowcmd", self._on_safe, qos_profile_sensor_data)
        self.create_subscription(LowCmd, "/lowcmd_bare", self._on_bare, qos_profile_sensor_data)
        self.latest = None
        self.start_time = None
        self.origin = None
        self.forward_axis = None
        self.start_yaw = 0.0
        self.speed_mps = speed_mps
        self.limit_s = limit_s
        self.reason = None
        self.violation_since = [None, None, None]
        self.elapsed = 0.0
        self.distance = 0.0
        self.max_forward = 0.0
        self.lateral = 0.0
        self.heading_offset_deg = 0.0
        self.bare_messages = 0
        self.safe_messages = 0
        self.safe_holds = 0
        self.target_updates = 0
        self.last_safe_target = None
        self.min_height = None
        self.max_tilt = 0.0

    def _on_bare(self, _msg):
        self.bare_messages += 1

    def _on_safe(self, msg):
        self.safe_messages += 1
        if self.start_time is None:
            return
        motors = msg.motor_cmd[:12]
        target = tuple(motor.q for motor in motors)
        if self.last_safe_target is not None and target != self.last_safe_target:
            self.target_updates += 1
        self.last_safe_target = target
        if motors[0].kp == 40.0 and motors[0].kd == 2.0:
            self.safe_holds += 1

    def ready(self):
        return self.latest is not None and self.safe_messages > 0 and self.command_pub.get_subscription_count() > 0

    def start(self):
        self.start_time, _, _, _, yaw = self.latest
        self.start_yaw = yaw
        self.forward_axis = math.cos(yaw), math.sin(yaw)
        self.bare_messages = 0
        self.safe_messages = 0
        self._publish_speed()
        self.create_timer(0.1, self._publish_speed)

    def _publish_speed(self):
        command = Twist()
        command.linear.x = self.speed_mps
        self.command_pub.publish(command)

    def _on_pose(self, msg):
        self.latest = pose_sample(msg)
        if self.start_time is None or self.reason is not None:
            return

        stamp, position, roll, pitch, yaw = self.latest
        self.min_height = position.z if self.min_height is None else min(self.min_height, position.z)
        self.max_tilt = max(self.max_tilt, math.degrees(abs(roll)), math.degrees(abs(pitch)))
        violations = (position.z < 0.16, abs(roll) > math.radians(50), abs(pitch) > math.radians(50))
        for index, violated in enumerate(violations):
            if not violated:
                self.violation_since[index] = None
            elif self.violation_since[index] is None:
                self.violation_since[index] = stamp
            elif stamp - self.violation_since[index] > 0.5:
                self.reason = "fall"

        scored_time = stamp - self.start_time - WARMUP_S
        if scored_time < 0:
            return
        self.elapsed = scored_time
        if self.origin is None:
            self.origin = position.x, position.y
        dx = position.x - self.origin[0]
        dy = position.y - self.origin[1]
        self.distance = dx * self.forward_axis[0] + dy * self.forward_axis[1]
        self.max_forward = max(self.max_forward, self.distance)
        self.lateral = dy * self.forward_axis[0] - dx * self.forward_axis[1]
        self.heading_offset_deg = math.degrees(math.remainder(yaw - self.start_yaw, math.tau))
        if self.reason is None:
            if self.distance >= 3.0 and scored_time <= self.limit_s:
                self.reason = "reached_3m" if self.bare_messages else "no_policy_commands"
            elif scored_time >= self.limit_s:
                self.reason = "sim_time_limit"

    def report(self):
        return {
            "success": self.reason == "reached_3m",
            "reason": self.reason,
            "speed_mps": self.speed_mps,
            "distance_forward_m": round(self.distance, 3),
            "max_forward_m": round(self.max_forward, 3),
            "lateral_m": round(self.lateral, 3),
            "heading_offset_deg": round(self.heading_offset_deg, 2),
            "scored_sim_time_s": round(self.elapsed, 3),
            "min_height_m": None if self.min_height is None else round(self.min_height, 3),
            "max_abs_tilt_deg": round(self.max_tilt, 2),
            "lowcmd_bare_messages": self.bare_messages,
            "lowcmd_messages": self.safe_messages,
            "lowcmd_hold_messages": self.safe_holds,
            "lowcmd_target_updates": self.target_updates,
        }


def main():
    parser = argparse.ArgumentParser(description="Check one local T3 walking episode")
    parser.add_argument("--speed", type=float, default=SPEED_MPS)
    parser.add_argument("--sim-limit", type=float, default=LIMIT_S)
    args = parser.parse_args()
    if not 0.1 <= args.speed <= 0.6 or not 0 < args.sim_limit <= LIMIT_S:
        parser.error("speed must be 0.1–0.6 m/s and sim-limit must be within (0, 90] s")

    rclpy.init(args=[])
    node = WalkingCheck(args.speed, args.sim_limit)
    try:
        deadline = time.monotonic() + 20.0
        while not node.ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.ready():
            print(f"T3 local check started at {args.speed:g} m/s", flush=True)
            node.start()
            deadline = time.monotonic() + 180.0
            while node.reason is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.reason is None:
                node.reason = "wall_time_limit"
        else:
            node.reason = "ros_topics_not_ready"
        report = node.report()
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report["success"] else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
