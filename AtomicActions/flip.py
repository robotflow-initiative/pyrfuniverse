from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'flip',
        scene_file='Flip.json',
        left_init_joint_positions=[0, 0, 0, 0, 0, 0, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    env.close_gripper('right')

    right_pos = env.get_current_position('right')
    env.step('right', right_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))

    tmp_pos = right_pos.copy()
    tmp_pos[0] -= 0.07
    tmp_pos[1] = 0.767
    env.step('right', tmp_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))

    before_insert_pos = env.get_current_position('right')
    before_insert_pos[2] += 0.1
    env.step('right', before_insert_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))

    inserting_pos = env.get_current_position('right')
    inserting_pos[0] -= 0.05
    inserting_pos[1] += 0.1
    env.step('right', inserting_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))

    inserting_pos[0] += 0.5
    env.step('right', inserting_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))

    # # Vector3(0.546000004, 0.888999999, 0.944999993)
    # env.step('right', np.array([0.546000004, 0.95, 0.85]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.546000004, 0.87, 0.85]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.close_gripper('right')
    #
    # env.step('right', np.array([0.546000004, 0.95, 0.85]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    #
    # env.step('right', np.array([0.45, 0.95, 0.95]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.45, 0.87, 0.95]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    #
    # env.step('right', np.array([0.5, 0.88, 0.95]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.45, 0.88, 0.9]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.5, 0.88, 0.9]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.45, 0.88, 0.85]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.5, 0.88, 0.85]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.45, 0.88, 0.8]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.5, 0.88, 0.8]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.45, 0.88, 0.75]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.5, 0.88, 0.75]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.45, 0.88, 0.7]), orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))

    while True:
        env._step()