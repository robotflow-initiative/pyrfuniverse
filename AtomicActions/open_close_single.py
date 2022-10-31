from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'tamp',
        scene_file='OpenCloseSingle.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    env.step('right', np.array([0.17, 0.9, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    env.step('right', np.array([0.17, 0.8, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    env.close_gripper('right')


    env.step('right', np.array([0.18, 0.85, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 4]))

    env.step('right', np.array([0.18, 0.85, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.2, 0.85, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.23, 0.88, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.25, 0.9, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.28, 0.9, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.3, 0.92, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.33, 0.95, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.33, 0.97, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.step('right', np.array([0.36, 0.97, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    env.wait(100)

    env.step('right', np.array([0.18, 0.85, 1]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, pi / 2]))

    while True:
        env._step()
