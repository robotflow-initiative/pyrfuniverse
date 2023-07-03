from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math


if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'cover_bimanual',
        scene_file='CoverBimanual.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    env.double_step(
        left_pos=np.array([-0.2, 0.9, 0.757000029]),
        right_pos=np.array([0.2, 0.9, 0.757000029])
    )

    env.double_step(
        left_pos=np.array([-0.35, 0.63, 0.791000009]),
        right_pos=np.array([0.35, 0.63, 0.791000009])
    )

    env.double_close()

    env.double_step(
        left_pos=np.array([-0.35, 0.9, 0.9]),
        right_pos=np.array([0.35, 0.9, 0.9])
    )

    env.wait(150)

    env.double_step(
        left_pos=np.array([-0.35, 0.63, 0.791000009]),
        right_pos=np.array([0.35, 0.63, 0.791000009])
    )

    while True:
        env._step()