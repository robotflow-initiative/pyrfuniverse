from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
try:
    import gymnasium as gym
except ImportError:
    print("This feature requires gymnasium, please install with `pip install gymnasium`")
    raise


class RFUniverseGymWrapper(RFUniverseBaseEnv, gym.Env):
    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        assets: list = [],
        graphics: bool = True,
        port: int = 5004,
        proc_id=0,
        log_level=1,
        ext_attr: list[type(attr.BaseAttr)] = []
    ):
        RFUniverseBaseEnv.__init__(
            self,
            executable_file=executable_file,
            scene_file=scene_file,
            assets=assets,
            graphics=graphics,
            port=port,
            proc_id=proc_id,
            log_level=log_level,
            ext_attr=ext_attr
        )

    def close(self):
        RFUniverseBaseEnv.close(self)
