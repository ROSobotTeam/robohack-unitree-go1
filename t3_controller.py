import sys
from collections import Counter
from math import radians
from pathlib import Path
from time import perf_counter

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd, LowState

from go1_policy import N_JOINTS, Go1Policy
from safety_block import SafetyBlock


CORRECTED_STEP_RAD = radians(4.9)


class T3SafetyBlock(SafetyBlock):
    def __init__(self):
        super().__init__()
        self._validation_counts = Counter()
        self._validation_total_s = 0.0
        self._validation_calls = 0
        self.create_timer(2.0, self._log_validation_counts)

    def _rejection_reason(self, message, previous_target):
        start = perf_counter()
        reason = super()._rejection_reason(message, previous_target)
        if reason == "step_limit":
            # The pending command may be based on the target before the last check completed.
            # Recheck the corrected command, including the full transition for collisions.
            for motor, previous in zip(message.motor_cmd, previous_target):
                lower = previous - CORRECTED_STEP_RAD
                upper = previous + CORRECTED_STEP_RAD
                motor.q = float(min(upper, max(lower, motor.q)))
            self._validation_counts["step_corrected"] += 1
            reason = super()._rejection_reason(message, previous_target)
        self._validation_counts[reason or "valid"] += 1
        self._validation_total_s += perf_counter() - start
        self._validation_calls += 1
        return reason

    def _log_validation_counts(self):
        if self._validation_calls:
            mean_ms = self._validation_total_s * 1000 / self._validation_calls
            self.get_logger().info(f"T3 target validation: {dict(self._validation_counts)} mean_ms={mean_ms:.1f}")


class T3Controller(Node):
    def __init__(self):
        super().__init__(
            "go1_t3_controller",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self._policy = Go1Policy(Path(__file__).with_name("policy.npz"))
        self._command = None
        self._state = None
        self._accepted_target = None

        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, latest)
        self.create_subscription(LowState, "/lowstate", self._on_state, latest)
        self.create_subscription(LowCmd, "/lowcmd", self._on_accepted, latest)
        self._publisher = self.create_publisher(LowCmd, "/lowcmd_bare", latest)
        self.create_timer(self._policy.ctrl_dt, self._infer)

    def _on_command(self, msg):
        command = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._command = command if np.all(np.isfinite(command)) else None

    def _on_state(self, msg):
        self._state = msg

    def _on_accepted(self, msg):
        self._accepted_target = np.array([motor.q for motor in msg.motor_cmd[:N_JOINTS]])

    def _infer(self):
        if self._command is None or self._state is None or self._accepted_target is None:
            return

        imu = self._state.imu_state
        motors = self._state.motor_state[:N_JOINTS]
        q = np.array([motor.q for motor in motors])
        dq = np.array([motor.dq for motor in motors])
        if not np.all(np.isfinite(np.r_[imu.quaternion, imu.gyroscope, q, dq])):
            return

        # Anchor the rate limiter and the observation to the Safety Block output.
        self._policy.last_target = self._accepted_target
        self._policy.last_act = (
            self._accepted_target - self._policy.default_pose
        ) / self._policy.action_scale
        target = self._policy.step(imu.quaternion, imu.gyroscope, q, dq, self._command)

        message = LowCmd()
        for motor, angle in zip(message.motor_cmd, target):
            motor.mode = 0x0A
            motor.q = float(angle)
            motor.kp = self._policy.kp
            motor.kd = self._policy.kd
        self._publisher.publish(message)


def main():
    role, = sys.argv[1:]
    rclpy.init(args=[])
    node = None
    try:
        node = T3SafetyBlock() if role == "safety" else T3Controller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if isinstance(node, SafetyBlock):
            node.close()
        elif node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
