from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

try:
    import gym
except ImportError:
    print("This feature requires gym, please install with `pip install gym==0.21.0`")
    raise


class RFUniverseGymWrapper(RFUniverseBaseEnv, gym.Env):
    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        assets: list = [],
        **kwargs
    ):
        RFUniverseBaseEnv.__init__(
            self,
            executable_file=executable_file,
            scene_file=scene_file,
            assets=assets,
            **kwargs,
        )

    def close(self):
        RFUniverseBaseEnv.close(self)
