from envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == "__main__":
    env = ToborRobotiq85ManipulationEnv(
        "tamp",
        scene_file="DropBimaunal.json",
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0],
    )
    env.reset()

    pi = math.pi

    env.double_step(
        left_pos=np.array([-0.06, 0.81, 0.85]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.08, 0.81, 0.85]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_close()

    env.double_step(
        left_pos=np.array([-0.06, 0.9, 0.85]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.08, 0.9, 0.85]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_step(
        left_pos=np.array([-0.06 + 0.165, 0.9, 0.85 - 0.25]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.08 + 0.165, 0.9, 0.85 - 0.25]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_step(
        left_pos=np.array([-0.06 + 0.165, 0.7, 0.85 - 0.25]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.08 + 0.165, 0.7, 0.85 - 0.25]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_open()

    # env.close()

    while True:
        env._step()
