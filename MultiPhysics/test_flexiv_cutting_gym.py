from envs.flexiv_cutting import FlexivCuttingEnv

env = FlexivCuttingEnv()
env.reset()

while 1:
    obs, reward, done, info = env.step(env.action_space.sample())
    if done:
        env.reset()
