from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math


if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'tamp',
        scene_file='GraspBimanual.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    left_pos = env.get_current_position('left')
    right_pos = env.get_current_position('right')

    env.double_step(
        left_pos=left_pos,
        left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
        right_pos=right_pos,
        right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    )

    env.double_step(
        left_pos=np.array([-0.65, 0.95, 1.076]),
        left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
        right_pos=np.array([0.1, 0.9157, 1.076]),
        right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    )

    env.double_step(
        left_pos=np.array([-0.394300014, 0.908999979, 0.91900003]),
        left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
        right_pos=np.array([-0.0300000003, 0.908999979, 0.91900003]),
        right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    )

    env.double_close()

    env.double_step(
        left_pos=np.array([-0.394300014 + 0.5, 1, 0.91900003]),
        left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
        right_pos=np.array([-0.0300000003 + 0.5, 1, 0.91900003]),
        right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    )

    env.double_step(
        left_pos=np.array([-0.394300014 + 0.5, 0.908999979, 0.91900003]),
        left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
        right_pos=np.array([-0.0300000003 + 0.5, 0.908999979, 0.91900003]),
        right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    )

    env.double_open()

    env.double_step(
        left_pos=np.array([-0.65 + 0.5, 0.95, 1.076]),
        left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
        right_pos=np.array([0.1 + 0.5, 0.95, 1.076]),
        right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    )

    # env.double_step(
    #     left_pos=left_pos,
    #     left_orn=p.getQuaternionFromEuler([0, -pi / 2, pi / 6]),
    #     right_pos=right_pos,
    #     right_orn=p.getQuaternionFromEuler([0, pi / 2, -pi / 6])
    # )

    while True:
        env._step()
