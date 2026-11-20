import csv
import math
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd, LowState

JOINT_NAMES = [
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
]

Q_MIN = np.array([-0.863, -0.686, -2.818] * 4, dtype=np.float64)
Q_MAX = np.array([0.863, 4.501, -0.888] * 4, dtype=np.float64)
DEFAULT_SAFE_Q = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)

KP_MIN, KP_MAX = 0.0, 1000.0
KD_MIN, KD_MAX = 0.0, 100.0
TAU_MIN, TAU_MAX = -100.0, 100.0
DQ_MIN, DQ_MAX = -20.0, 20.0

MAX_STEP_RAD = math.radians(4.99)
INTERP_STEP_RAD = math.radians(0.1)
MAX_PENETRATION_M = 0.001
WATCHDOG_TIMEOUT_SEC = 0.45
PUBLISH_RATE_HZ = 100.0

MODEL_PATH = Path('/opt/go1/models/unitree_go1/scene.xml')
PAIRS_CSV_PATH = Path('/opt/go1/models/unitree_go1/t2_collision_pairs.csv')


def load_collision_pairs(csv_path: Path, logger) -> set[tuple[int, int]]:
    pairs: set[tuple[int, int]] = set()
    with csv_path.open('r', newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            if row['diagnostic_policy'].strip() != 'checked':
                continue
            id1 = int(row['geom1_id'])
            id2 = int(row['geom2_id'])
            pairs.add((min(id1, id2), max(id1, id2)))

    logger.info(f'Loaded {len(pairs)} collision pairs from {csv_path}')
    return pairs


class SafetyBlockNode(Node):
    def __init__(self):
        super().__init__('safety_block')

        self.mj_model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.mj_data = mujoco.MjData(self.mj_model)
        self.qpos_indices = self._resolve_qpos_indices()
        self.check_pairs = load_collision_pairs(PAIRS_CSV_PATH, self.get_logger())

        self.has_received_cmd = False
        self.has_state = False
        self.safe_q = DEFAULT_SAFE_Q.copy()
        self.safe_dq = np.zeros(12, dtype=np.float64)
        self.safe_kp = np.full(12, 80.0, dtype=np.float64)
        self.safe_kd = np.full(12, 2.0, dtype=np.float64)
        self.safe_tau = np.zeros(12, dtype=np.float64)
        self.safe_mode = [0x0A] * 12
        self.last_cmd_time = None

        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_cmd = self.create_subscription(LowCmd, '/lowcmd_bare', self.on_lowcmd_bare, 10)
        self.sub_state = self.create_subscription(LowState, '/lowstate', self.on_lowstate, state_qos)
        self.pub_cmd = self.create_publisher(LowCmd, '/lowcmd', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.on_timer)
        self.get_logger().info('Safety Block node started (100 Hz, sim_time watchdog=0.45s)')

    def _resolve_qpos_indices(self) -> list[int]:
        indices = []
        for name in JOINT_NAMES:
            jnt_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id < 0:
                short_name = name.replace('_joint', '')
                jnt_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, short_name)
            if jnt_id < 0:
                raise RuntimeError(f'Joint {name} not found in {MODEL_PATH}')
            indices.append(int(self.mj_model.jnt_qposadr[jnt_id]))
        return indices

    def _is_pose_collision_free(self, q: np.ndarray) -> bool:
        for i, qpos_idx in enumerate(self.qpos_indices):
            self.mj_data.qpos[qpos_idx] = float(q[i])
        mujoco.mj_kinematics(self.mj_model, self.mj_data)
        mujoco.mj_collision(self.mj_model, self.mj_data)

        for k in range(self.mj_data.ncon):
            contact = self.mj_data.contact[k]
            pair = (min(int(contact.geom1), int(contact.geom2)),
                    max(int(contact.geom1), int(contact.geom2)))
            if pair in self.check_pairs and contact.dist < -MAX_PENETRATION_M:
                return False
        return True

    def _is_transition_safe(self, q_start: np.ndarray, q_end: np.ndarray) -> bool:
        if not self._is_pose_collision_free(q_end):
            return False
        max_diff = float(np.max(np.abs(q_end - q_start)))
        steps = max(1, int(math.ceil(max_diff / INTERP_STEP_RAD)))
        for s in range(1, steps):
            alpha = s / steps
            q_mid = q_start + alpha * (q_end - q_start)
            if not self._is_pose_collision_free(q_mid):
                return False
        return True

    def on_lowstate(self, msg: LowState):
        if not self.has_state and not self.has_received_cmd:
            q_meas = np.array([float(msg.motor_state[i].q) for i in range(12)], dtype=np.float64)
            if np.all(np.isfinite(q_meas)):
                q_clamped = np.clip(q_meas, Q_MIN, Q_MAX)
                if self._is_pose_collision_free(q_clamped):
                    self.safe_q = q_clamped
            self.has_state = True

    def on_lowcmd_bare(self, msg: LowCmd):
        q = np.empty(12, dtype=np.float64)
        dq = np.empty(12, dtype=np.float64)
        kp = np.empty(12, dtype=np.float64)
        kd = np.empty(12, dtype=np.float64)
        tau = np.empty(12, dtype=np.float64)
        modes = [0x0A] * 12

        for i in range(12):
            mc = msg.motor_cmd[i]
            vals = (float(mc.q), float(mc.dq), float(mc.kp), float(mc.kd), float(mc.tau))
            if not all(math.isfinite(v) for v in vals):
                self.get_logger().warn('Rejected command: NaN/Inf detected', throttle_duration_sec=1.0)
                return
            q[i], dq[i], kp[i], kd[i], tau[i] = vals
            modes[i] = int(mc.mode)

        if np.any(q < Q_MIN) or np.any(q > Q_MAX):
            self.get_logger().warn('Rejected command: joint angle out of bounds', throttle_duration_sec=1.0)
            return

        if (np.any(dq < DQ_MIN) or np.any(dq > DQ_MAX) or
                np.any(kp < KP_MIN) or np.any(kp > KP_MAX) or
                np.any(kd < KD_MIN) or np.any(kd > KD_MAX) or
                np.any(tau < TAU_MIN) or np.any(tau > TAU_MAX)):
            self.get_logger().warn('Rejected command: dq/kp/kd/tau out of bounds', throttle_duration_sec=1.0)
            return

        if np.any(np.abs(q - self.safe_q) > MAX_STEP_RAD):
            self.get_logger().warn('Rejected command: step > 5 deg', throttle_duration_sec=1.0)
            return

        if not self._is_transition_safe(self.safe_q, q):
            self.get_logger().warn('Rejected command: self-collision detected', throttle_duration_sec=1.0)
            return

        self.safe_q = q
        self.safe_dq = dq
        self.safe_kp = kp
        self.safe_kd = kd
        self.safe_tau = tau
        self.safe_mode = modes
        self.has_received_cmd = True
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        hold_mode = True
        if self.last_cmd_time is not None and now.nanoseconds > 0:
            elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
            if elapsed < WATCHDOG_TIMEOUT_SEC:
                hold_mode = False

        out = LowCmd()
        for i in range(12):
            out.motor_cmd[i].q = float(self.safe_q[i])
            if hold_mode:
                out.motor_cmd[i].mode = 0x0A
                out.motor_cmd[i].dq = 0.0
                out.motor_cmd[i].tau = 0.0
                out.motor_cmd[i].kp = float(self.safe_kp[i]) if self.safe_kp[i] > 0.0 else 80.0
                out.motor_cmd[i].kd = float(self.safe_kd[i]) if self.safe_kd[i] >= 0.0 else 2.0
            else:
                out.motor_cmd[i].mode = int(self.safe_mode[i])
                out.motor_cmd[i].dq = float(self.safe_dq[i])
                out.motor_cmd[i].tau = float(self.safe_tau[i])
                out.motor_cmd[i].kp = float(self.safe_kp[i])
                out.motor_cmd[i].kd = float(self.safe_kd[i])

        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBlockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
