from csv import DictReader
from math import ceil, radians
from pathlib import Path

import mujoco
import numpy as np

JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)
SAMPLE_STEP_RAD = radians(0.1)
MAX_PENETRATION_M = 0.001


class CollisionChecker:
    def __init__(self, model_dir):
        directory = Path(model_dir)
        self.model = mujoco.MjModel.from_xml_path(str(directory / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.qpos_addresses = [
            int(self.model.jnt_qposadr[self.model.joint(name).id])
            for name in JOINT_NAMES
        ]

        with (directory / "t2_collision_pairs.csv").open(
            encoding="utf-8-sig", newline=""
        ) as table:
            self.checked_pairs = {
                tuple(sorted((int(row["geom1_id"]), int(row["geom2_id"]))))
                for row in DictReader(table)
                if row["diagnostic_policy"] == "checked"
            }

        if len(self.checked_pairs) != 655 or any(
            not 0 <= first < second < self.model.ngeom
            for first, second in self.checked_pairs
        ):
            raise ValueError("Invalid Go1 collision pair table")

        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.home_target = tuple(self.data.qpos[self.qpos_addresses])

    def pose_is_clear(self, target):
        self.data.qpos[self.qpos_addresses] = target
        mujoco.mj_fwdPosition(self.model, self.data)
        for contact in self.data.contact:
            if contact.dist < -MAX_PENETRATION_M:
                pair = tuple(sorted(map(int, contact.geom)))
                if pair in self.checked_pairs:
                    return False
        return True

    def transition_is_clear(self, previous_target, target):
        start = np.asarray(previous_target, dtype=float)
        end = np.asarray(target, dtype=float)
        max_change = float(np.max(np.abs(end - start)))
        intervals = ceil(max_change / SAMPLE_STEP_RAD)
        return all(
            self.pose_is_clear(pose)
            for pose in np.linspace(start, end, intervals + 1)
        )
