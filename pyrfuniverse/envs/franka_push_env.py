from pyrfuniverse.envs import RFUniverseBaseEnv
from pyrfuniverse.utils import RFUniverseController
import numpy as np


class FrankaPushEnv(RFUniverseBaseEnv):
    def __init__(self, executable_file=None):
        super().__init__(
            executable_file,
            camera_channel=True,
            rigidbody_channel=True,
            articulation_channel=True,
        )

        self.prev_joint_positions = np.array([0.0 for i in range(8)], dtype=float)
        self.ik_controller = RFUniverseController("franka")

    def step(self, a: np.ndarray):
        """
        Params:
            a: 4d numpy array. The first 3 dimensions are for the unity-position of Franka grasp point,
               while the 4th dimension is for gripper's width.
        """
        assert a.shape == (4,), "The shape of action must be (4,), but got {}".format(
            a.shape
        )

        eef_pos = a[0:3]
        joint_positions = self.ik_controller.calculate_ik(eef_pos)
        joint_positions.append(float(a[3]))
        velocities = [1.0 for i in range(8)]
        a = np.array(joint_positions + velocities)

        self._set_franka_joints(a)
        self._wait_for_moving()
        self.t += 1

        self._update_joint_positions()
        return self._get_obs()

    def _get_obs(self):
        # Read message from channels
        return self.rigidbody_channel.data

    def reset(self):
        self.t = 0
        self.r = 0
        self.env.reset()

        return self._get_obs()

    def seed(self):
        pass

    def _set_franka_joints(self, a: np.ndarray):
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=0,
            joint_positions=list(a[0:7]),
            speed_scales=list(a[8:15]),
        )
        self._step()

        # Since a[7] is gripper's width, convert it to joint position value
        a[7] = -1 * a[7] / 2

        self.articulation_channel.set_action(
            "SetJointPosition",
            index=1,
            joint_positions=[a[7], a[7]],
            speed_scales=[a[15], a[15]],
        )
        self._step()

    def _wait_for_moving(self):
        self.articulation_channel.data.clear()
        while not 0 in self.articulation_channel.data.keys():
            self._step()
        while not (
            self.articulation_channel.data[0]["all_stable"]
            and self.articulation_channel.data[1]["all_stable"]
        ):
            self._step()

    def _update_joint_positions(self):
        data = self.articulation_channel.data
        arm_joint_positions = data[0]["joint_positions"]
        gripper_joint_positions = data[1]["joint_positions"]

        self.prev_joint_positions[0:7] = np.array(arm_joint_positions)
        self.prev_joint_positions[7] = abs(gripper_joint_positions[0]) + abs(
            gripper_joint_positions[1]
        )

    def _calculate_speed(self, a: np.ndarray):
        relative_joint_positions = abs(a[0:7] - self.prev_joint_positions[0:7])
        speed = relative_joint_positions / relative_joint_positions.min()
        speed = np.concatenate(speed, np.array([1.0]))

        return speed
