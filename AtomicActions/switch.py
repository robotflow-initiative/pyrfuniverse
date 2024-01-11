from envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
import numpy as np
import pybullet as p
import math

if __name__ == "__main__":
    env = ToborRobotiq85ManipulationEnv(
        "tamp",
        scene_file="Switch.json",
        left_init_joint_positions=[-90, 45, 0, 75, 0, 60, 0],
        right_init_joint_positions=[-90, -45, 0, -75, 0, -60, 0],
    )
    env.reset()

    pi = math.pi

    right_pos = env.get_current_position("right")
    env.step("right", right_pos, orientation=p.getQuaternionFromEuler([0, 0, 0]))
    env.close_gripper("right")

    # Vector3(0.0392000005, 1.18980002, 1.12310004)

    env.step(
        "right",
        np.array([0.0392000005, 1.18980002, 1.12310004]),
        orientation=p.getQuaternionFromEuler([0, 0, 0]),
    )

    env.step(
        "right",
        np.array([0.0392000005, 1.18980002, 0.9]),
        orientation=p.getQuaternionFromEuler([0, 0, 0]),
    )

    env.wait(20)

    env.step(
        "right",
        np.array([0.0392000005, 1.23, 1.12310004]),
        orientation=p.getQuaternionFromEuler([0, 0, 0]),
    )

    env.step(
        "right",
        np.array([0.0392000005, 1.23, 0.9]),
        orientation=p.getQuaternionFromEuler([0, 0, 0]),
    )

    while True:
        env._step()
