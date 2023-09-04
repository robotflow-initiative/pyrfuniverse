from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import (
    ToborRobotiq85ManipulationEnv,
)
import numpy as np
import pybullet as p
import math

if __name__ == "__main__":
    env = ToborRobotiq85ManipulationEnv(
        "stretch_bimanual",
        scene_file="StretchBimanual.json",
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0],
    )
    env.reset()

    pi = math.pi

    left_pos = env.get_current_position("left")
    right_pos = env.get_current_position("right")

    env.double_step(
        left_pos=left_pos,
        right_pos=right_pos,
    )

    env.double_step(
        left_pos=np.array([-0.200000048, 0.7, 0.957000017]),
        right_pos=np.array([0.200000048, 0.7, 0.957000017]),
    )

    env.double_step(
        left_pos=np.array([-0.200000048, 0.613999963, 0.957000017]),
        right_pos=np.array([0.200000048, 0.613999963, 0.957000017]),
    )

    env.double_close()

    env.double_step(
        left_pos=np.array([-0.200000048, 0.7, 0.957000017]),
        right_pos=np.array([0.200000048, 0.7, 0.957000017]),
    )

    env.double_step(
        left_pos=np.array([-0.4, 0.8, 0.957000017]),
        right_pos=np.array([0.4, 0.8, 0.957000017]),
    )

    env.double_step(
        left_pos=np.array([-0.200000048, 0.7, 0.957000017]),
        right_pos=np.array([0.200000048, 0.7, 0.957000017]),
    )

    env.double_step(
        left_pos=np.array([-0.4, 0.8, 0.957000017]),
        right_pos=np.array([0.4, 0.8, 0.957000017]),
    )

    while True:
        env._step()
