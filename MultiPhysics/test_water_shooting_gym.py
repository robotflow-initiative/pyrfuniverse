from pyrfuniverse.envs.multi_physics import UR5WaterShootingEnv

env = UR5WaterShootingEnv(urdf_file="../URDF/UR5_robotiq_85/ur5_robotiq_85.urdf")
env.reset()

while 1:
    obs, reward, done, info = env.step(env.action_space.sample())
    if done:
        env.reset()
