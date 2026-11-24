from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd

from collision_checks import CollisionChecker
from safety_checks import basic_rejection_reason

PUBLISH_PERIOD_S = 0.01
HOLD_AFTER_NS = 450_000_000


def joint_targets(message):
    return tuple(motor.q for motor in message.motor_cmd[:12])


def hold_command(target):
    message = LowCmd()
    for motor, angle in zip(message.motor_cmd, target):
        motor.mode = 0x0A
        motor.q = float(angle)
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = 40.0
        motor.kd = 2.0
    return message


def is_fresh(stamp_ns, now_ns):
    return stamp_ns is not None and 0 <= now_ns - stamp_ns < HOLD_AFTER_NS


class SafetyBlock(Node):
    def __init__(self):
        super().__init__(
            "safety_block",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        model_dir = self.declare_parameter("model_dir", "/opt/go1/model").value
        self._checker = CollisionChecker(model_dir)

        self._hold = hold_command(self._checker.home_target)
        home = joint_targets(self._hold)
        if (
            basic_rejection_reason(self._hold.motor_cmd, home) is not None
            or not self._checker.pose_is_clear(home)
        ):
            raise RuntimeError("Unsafe Go1 home target")

        self._output = self._hold
        self._last_input_ns = None
        self._pending = None
        self._job = None
        self._worker = ThreadPoolExecutor(max_workers=1)

        self._publisher = self.create_publisher(LowCmd, "/lowcmd", 10)
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LowCmd, "/lowcmd_bare", self._on_command, input_qos)
        self.create_timer(PUBLISH_PERIOD_S, self._publish_next)

    def _on_command(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns > 0:
            self._last_input_ns = now_ns
            self._pending = (now_ns, message)

    def _rejection_reason(self, message, previous_target):
        reason = basic_rejection_reason(message.motor_cmd, previous_target)
        if reason is not None:
            return reason

        target = joint_targets(message)
        if target != previous_target and not self._checker.transition_is_clear(
            previous_target, target
        ):
            return "collision"
        return None

    def _finish_check(self, now_ns):
        if self._job is None:
            return
        stamp_ns, message, future = self._job
        if not future.done():
            return

        self._job = None
        if not is_fresh(stamp_ns, now_ns):
            return

        try:
            reason = future.result()
        except Exception as error:
            self.get_logger().error(f"Command validation failed: {error}")
            reason = "validation_error"

        if reason is None:
            self._hold = hold_command(joint_targets(message))
            self._output = message
        elif reason != "nonfinite":
            self._output = self._hold

    def _publish_next(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            return

        self._finish_check(now_ns)
        if not is_fresh(self._last_input_ns, now_ns):
            self._pending = None
            self._output = self._hold

        self._publisher.publish(self._output)

        if self._job is None and self._pending is not None:
            stamp_ns, message = self._pending
            self._pending = None
            future = self._worker.submit(
                self._rejection_reason, message, joint_targets(self._output)
            )
            self._job = (stamp_ns, message, future)

    def close(self):
        self._worker.shutdown(wait=True)
        self.destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SafetyBlock()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
