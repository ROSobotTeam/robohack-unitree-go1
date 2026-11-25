from math import isfinite, radians

JOINT_LIMITS = ((-0.863, 0.863), (-0.686, 4.501), (-2.818, -0.888)) * 4
MAX_STEP_RAD = radians(5)


def basic_rejection_reason(motors, previous_target):
    if len(motors) != 20 or len(previous_target) != 12:
        return "size"

    for motor in motors:
        fields = (motor.q, motor.dq, motor.kp, motor.kd, motor.tau)
        if not all(isfinite(value) for value in fields):
            return "nonfinite"

    for motor, previous, (lower, upper) in zip(
        motors[:12], previous_target, JOINT_LIMITS
    ):
        if not (
            0.0 <= motor.kp <= 1000.0
            and 0.0 <= motor.kd <= 100.0
            and -100.0 <= motor.tau <= 100.0
            and -20.0 <= motor.dq <= 20.0
        ):
            return "control_limit"
        if not lower <= motor.q <= upper:
            return "angle_limit"
        if not isfinite(previous) or abs(motor.q - previous) > MAX_STEP_RAD:
            return "step_limit"

    return None
