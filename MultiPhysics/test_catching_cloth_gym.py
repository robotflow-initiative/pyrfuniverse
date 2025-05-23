from envs.kinova_gen2_catching_cloth_env import KinovaGen2CatchingClothEnv

env = KinovaGen2CatchingClothEnv(urdf_file="../URDF/jaco/j2s7s300_gym.urdf")
env.reset()

while 1:
    obs, reward, done, info = env.step(env.action_space.sample())
    if done:
        env.reset()
