from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'tamp',
        scene_file='OpenCloseBimanual.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    left_pos = env.get_current_position('left')
    env.step('left', left_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))
    env.step('left', np.array([0, 0.82, 0.65]),
             orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))
    env.step('left', np.array([0, 0.82, 0.75]),
             orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))
    env.close_gripper('left')

    env.step('right', np.array([0, 1, 0.73]))
    env.step('right', np.array([0.00209999993, 0.904300025, 0.734399974]))
    env.close_gripper('right')
    for i in range(15):
        env.step('right', np.array([0.00209999993, 0.92, 0.734399974]),
                 orientation=p.getQuaternionFromEuler([pi / 2, pi / 14 * i, 0]))
    env.step('right', np.array([0, 1, 0.734]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi, 0]))

    env.wait(100)

    env.step('right', np.array([0.00209999993, 0.92, 0.734399974]),
             orientation=p.getQuaternionFromEuler([pi / 2, pi, 0]))
    for i in range(15):
        env.step('right', np.array([0.00209999993, 0.92, 0.734399974]),
                 orientation=p.getQuaternionFromEuler([pi / 2, pi - pi / 14 * i, 0]))
    # env.open_gripper('right')
    # env.step('right', np.array([0, 1, 0.75]))
    #
    while True:
        env._step()

# Vector3(0,0.830799997,0.975000024)