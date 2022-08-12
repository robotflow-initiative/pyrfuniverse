from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math


if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'tamp',
        scene_file='PlugInOut.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    left_pos = env.get_current_position('left')
    right_pos = env.get_current_position('right')

    env.double_step(
        left_pos=left_pos,
        right_pos=right_pos,
    )

    env.step('right', np.array([0.7, 0.83, 0.74]))
    env.close_gripper('right')
    env.step('right', right_pos)

    env.step('left', np.array([-0.0801000004, 0.95, 0.793799996]))
    env.close_gripper('left')
    env.step('left', np.array([-0.2, 0.95, 0.793799996]))

    env.wait(50)

    env.step('left', np.array([-0.0601000004, 0.95, 0.793799996]))

    while True:
        env._step()