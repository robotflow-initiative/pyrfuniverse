from envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == "__main__":
    env = ToborRobotiq85ManipulationEnv(
        "tamp",
        scene_file="Hammering.json",
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0],
    )
    env.reset()

    pi = math.pi

    # right_pos = env.get_current_position('right')
    # env.step('right', right_pos,
    #          orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))

    env.step(
        "right",
        np.array([0.85, 0.9, 0.7]),
        orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.85, 0.8, 0.7]),
        orientation=p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.close_gripper("right")

    # for i in range(5):
    #     env.step('right', np.array([0.85, 0.8, 0.7]),
    #              orientation=p.getQuaternionFromEuler(
    #                  [pi / 2 - pi / 10 * (i + 1), pi / 2 - pi / 10 * (i + 1), -pi / 10 * (i + 1)]
    #              )
    #     )

    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.71, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    env.step(
        "right",
        np.array([0.444999993, 0.67, 0.67]),
        p.getQuaternionFromEuler([pi / 2, pi / 2, 0]),
    )
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.370999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))
    # env.step('right', np.array([0.26699999, 0.420999992, 0.751999974]), p.getQuaternionFromEuler([pi / 2, pi / 2, 0]))

    while True:
        env._step()
