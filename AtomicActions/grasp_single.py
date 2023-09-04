from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import (
    ToborRobotiq85ManipulationEnv,
)
import numpy as np
import pybullet as p
import math

if __name__ == "__main__":
    env = ToborRobotiq85ManipulationEnv(
        "tamp",
        scene_file="GraspSingle.json",
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0],
    )
    env.reset()

    pi = math.pi

    env.step("left", np.array([-0.35, 1.1, 1]))
    env.step("left", np.array([-0.35, 0.8, 1]))
    env.close_gripper("left")
    env.step("left", np.array([0.113, 1.2, 0.85]))
    env.step("left", np.array([0.113, 0.98, 0.85]))
    env.open_gripper("left")
    env.step("left", np.array([0.113, 1.2, 0.85]))
    env.step("left", np.array([-0.35, 1.1, 1]))

    env.close()

    # env.step('left', np.array([-0.85, 0.8, 0.81]), p.getQuaternionFromEuler([pi / 2, 0, 0]))

    # env.close_gripper('left')
    # for i in range(10):
    #     env.step('left', np.array([-0.85, 1.0, 0.81]), p.getQuaternionFromEuler([pi / 2, 0, -pi / 20 * (i + 1)]))
    #
    # env.step('left', np.array([0, 1.0, 0.81]), p.getQuaternionFromEuler([pi / 2, 0, -pi / 2]))

    while True:
        env._step()
