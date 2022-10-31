from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'tamp',
        scene_file='DropBimaunal.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    # Vector3(0.248999998,0.771799982,0.815500021)

    # Vector3(0,0.771799982,0.885499716)

    # Vector3(0.165000007,0.0480000004,0.644999981)

    env.double_step(
        left_pos=np.array([-0.0666000023, 0.82, 0.881600022]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.0666000023, 0.82, 0.881600022]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_close()

    env.double_step(
        left_pos=np.array([-0.0666000023, 0.9, 0.881600022]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.0666000023, 0.9, 0.881600022]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_step(
        left_pos=np.array([-0.0666000023 + 0.165000007, 0.9, 0.881600022 - 0.17]),
        left_orn=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
        right_pos=np.array([0.0666000023 + 0.165000007, 0.9, 0.881600022 - 0.17]),
        right_orn=p.getQuaternionFromEuler([pi / 2, -pi / 2, 0]),
    )

    env.double_open()

    # env.close()

    while True:
        env._step()