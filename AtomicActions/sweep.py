from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import (
    ToborRobotiq85ManipulationEnv,
)
import numpy as np
import pybullet as p
import math

if __name__ == "__main__":
    env = ToborRobotiq85ManipulationEnv(
        "sweep",
        scene_file="Sweep.json",
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0],
    )
    env.reset()

    pi = math.pi

    right_pos = env.get_current_position("right")
    env.step("right", right_pos, orientation=p.getQuaternionFromEuler([0, 0, pi / 2]))
    env.step(
        "right",
        np.array([0.495000005, 0.632000029, 0.7]),
        orientation=p.getQuaternionFromEuler([0, 0, pi / 2]),
    )
    env.step(
        "right",
        np.array([0.495000005, 0.632000029, 0.804499984]),
        orientation=p.getQuaternionFromEuler([0, 0, pi / 2]),
    )
    # Vector3(0.495000005, 0.632000029, 0.804499984)

    env.close_gripper("right")
    env.step(
        "right",
        np.array([0.495000005, 0.9, 0.804499984]),
        orientation=p.getQuaternionFromEuler([0, 0, pi / 2]),
    )
    env.step(
        "right",
        np.array([0.495000005, 0.9, 0.804499984]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    env.step(
        "right",
        np.array([0, 1, 0.804499984]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    env.step(
        "right",
        np.array([0, 0.8, 0.804499984]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    env.step(
        "right",
        np.array([0.5, 0.8, 0.804499984]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    env.step(
        "right",
        np.array([0, 0.9, 0.9]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    env.step(
        "right",
        np.array([0, 0.8, 0.9]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    env.step(
        "right",
        np.array([0.5, 0.8, 0.9]),
        orientation=p.getQuaternionFromEuler([pi / 2, 0, 0]),
    )

    while True:
        env._step()
