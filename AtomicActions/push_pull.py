from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == '__main__':
    env = ToborRobotiq85ManipulationEnv(
        'tamp',
        scene_file='PushPull.json',
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0]
    )
    env.reset()

    pi = math.pi

    left_pos = env.get_current_position('left')
    env.step('left', left_pos, orientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Vector3(-0.319999993, 1.09619999, 1.07000005)

    # Open
    # Highest level
    env.step('left', np.array([-0.319999993, 1.3, 1.1000005]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.close_gripper('left')
    env.step('left', np.array([-0.319999993, 1.3, 1.0]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.open_gripper('left')
    env.step('left', np.array([-0.319999993, 1.09619999, 0.8]), orientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Middle level
    env.step('left', np.array([-0.319999993, 1.09619999, 1.1000005]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.close_gripper('left')
    env.step('left', np.array([-0.319999993, 1.09619999, 0.95]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.open_gripper('left')
    env.step('left', np.array([-0.319999993, 0.930000007, 0.8]), orientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Lowest level
    env.step('left', np.array([-0.319999993, 0.930000007, 1.1000005]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.close_gripper('left')
    env.step('left', np.array([-0.319999993, 0.930000007, 0.9]), orientation=p.getQuaternionFromEuler([0, 0, 0]))

    env.wait(30)
    # Close
    # Lowest level
    env.step('left', np.array([-0.319999993, 0.930000007, 1.1000005]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.open_gripper('left')
    env.step('left', np.array([-0.319999993, 0.930000007, 0.8]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.step('left', np.array([-0.319999993, 1.09619999, 0.8]), orientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Middle level
    env.step('left', np.array([-0.319999993, 1.09619999, 0.95]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.close_gripper('left')
    env.step('left', np.array([-0.319999993, 1.09619999, 1.1000005]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.open_gripper('left')
    env.step('left', np.array([-0.319999993, 1.09619999, 0.8]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.step('left', np.array([-0.319999993, 1.32, 0.8]), orientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Highest level
    env.step('left', np.array([-0.319999993, 1.28, 1.0]), orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.close_gripper('left')
    env.step('left', np.array([-0.319999993, 1.28, 1.1000005]), orientation=p.getQuaternionFromEuler([0, 0, 0]))

    while True:
        env._step()


# Vector3(-0.319999993,0.930000007,1.07000005)