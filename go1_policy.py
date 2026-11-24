"""Numpy-реализация обученной RL-политики Go1 для задания T3.

Один и тот же файл используется:
  * в Colab для проверки в симуляторе хакатона (sim2sim);
  * в Docker-образе решения внутри ROS 2 узла (без JAX, без GPU).

Файл весов policy.npz создаётся ноутбуком обучения.
"""
import numpy as np

# Порядок суставов (как в LowCmd/LowState Unitree и в модели MuJoCo):
# FR(hip, thigh, calf), FL(...), RR(...), RL(...)
N_JOINTS = 12


def gravity_from_quat(quat_wxyz):
    """Вектор гравитации [0, 0, -1] в системе корпуса по кватерниону (w, x, y, z)."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    # третья строка матрицы поворота R (корпус -> мир); g_body = R^T [0,0,-1] = -R[2, :]
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return -np.array([r20, r21, r22])


def _swish(x):
    return x / (1.0 + np.exp(-x))


class Go1Policy:
    def __init__(self, npz_path):
        z = np.load(npz_path, allow_pickle=False)
        n = int(z["n_layers"])
        self.W = [z[f"W{i}"].astype(np.float64) for i in range(n)]
        self.b = [z[f"b{i}"].astype(np.float64) for i in range(n)]
        self.obs_mean = z["obs_mean"].astype(np.float64)
        self.obs_std = z["obs_std"].astype(np.float64)
        self.default_pose = z["default_pose"].astype(np.float64)
        self.action_scale = float(z["action_scale"])
        self.max_delta = float(z["max_delta"])
        self.q_lo = z["q_lo"].astype(np.float64)
        self.q_hi = z["q_hi"].astype(np.float64)
        self.kp = float(z["kp"])
        self.kd = float(z["kd"])
        self.ctrl_dt = float(z["ctrl_dt"])
        # Часы походки (есть, если политика обучалась с заданным аллюром).
        self.use_clock = "gait_freq_base" in z.files
        if self.use_clock:
            self.gait_freq_base = float(z["gait_freq_base"])
            self.gait_freq_slope = float(z["gait_freq_slope"])
        self.reset()

    def reset(self, start_target=None):
        """Сброс внутреннего состояния (в начале эпизода)."""
        self.last_act = np.zeros(N_JOINTS)
        self.phase = 0.0
        if start_target is None:
            self.last_target = self.default_pose.copy()
        else:
            self.last_target = np.asarray(start_target, dtype=np.float64).copy()

    def gait_freq(self, cmd):
        return self.gait_freq_base + self.gait_freq_slope * abs(float(cmd[0]))

    def build_obs(self, quat_wxyz, gyro, q, dq, cmd):
        """Наблюдение актора в том же порядке, что при обучении (45 чисел, +2 с часами походки)."""
        parts = [
            np.asarray(gyro, dtype=np.float64),          # 3: угловая скорость в системе корпуса
            gravity_from_quat(quat_wxyz),                # 3: гравитация в системе корпуса
            np.asarray(q, dtype=np.float64) - self.default_pose,  # 12
            np.asarray(dq, dtype=np.float64),            # 12
            self.last_act,                               # 12: последнее применённое действие
            np.asarray(cmd, dtype=np.float64),           # 3: (vx, vy, wz)
        ]
        if self.use_clock:
            parts.append(np.array([np.sin(2 * np.pi * self.phase), np.cos(2 * np.pi * self.phase)]))
        return np.concatenate(parts)

    def forward(self, obs):
        """Детерминированное действие сети в [-1, 1]."""
        x = (obs - self.obs_mean) / self.obs_std
        last = len(self.W) - 1
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            x = x @ W + b
            if i < last:
                x = _swish(x)
        return np.tanh(x[:N_JOINTS])  # первая половина выхода - среднее распределения

    def step(self, quat_wxyz, gyro, q, dq, cmd):
        """Один шаг политики (вызывать раз в ctrl_dt = 0.02 с). Возвращает 12 целевых углов.

        Ограничения те же, что при обучении и в Safety Block: изменение цели не больше
        max_delta за шаг и попадание в пределы суставов.
        """
        obs = self.build_obs(quat_wxyz, gyro, q, dq, cmd)
        a = self.forward(obs)
        target = self.default_pose + a * self.action_scale
        target = np.clip(target, self.last_target - self.max_delta, self.last_target + self.max_delta)
        target = np.clip(target, self.q_lo, self.q_hi)
        self.last_act = (target - self.default_pose) / self.action_scale
        self.last_target = target
        if self.use_clock:
            self.phase = (self.phase + self.ctrl_dt * self.gait_freq(cmd)) % 1.0
        return target.copy()
