from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'fold_single',
        scene_file='FoldSingle.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    env.step('right', np.array([0.2, 0.7, 0.757000029]))
    env.step('right', np.array([0.2, 0.64199996, 0.757000029]))
    env.close_gripper('right')
    env.step('right', np.array([0.2, 0.8, 0.757000029]))
    env.step('right', np.array([-0.1, 0.7, 1]))
    env.open_gripper('right')
    env.step('right', np.array([-0.1, 0.8, 1]))

    env.wait(150)

    env.step('right', np.array([-0.1, 0.642, 1]))
    env.close_gripper('right')
    env.step('right', np.array([0.2, 0.7, 0.757000029]))
    env.step('right', np.array([0.2, 0.64199996, 0.757000029]))
    env.open_gripper('right')

    while True:
        env._step()