from pyrfuniverse.envs.robotics.pick_and_place_env import FrankaPickAndPlaceEnv
import numpy as np


env = FrankaPickAndPlaceEnv(
    executable_file=None,
    asset_bundle_file=None,
    max_episode_length=50,
    reward_type='sparse',
    assets=['Rigidbody_Box']
)

env.reset()

for i in range(1000):
    env.step(action=np.random.uniform(low=-1, high=1, size=(4,)))

env.close()