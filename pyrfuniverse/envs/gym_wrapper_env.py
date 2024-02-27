from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
from typing import Any, SupportsFloat
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
        ext_attr: list[type(attr.BaseAttr)] = [],
        check_version: bool = True
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
            ext_attr=ext_attr,
            check_version=check_version
        )

    def env_step(self, count: int = 1):
        """
        Send the messages of called functions to Unity and simulate for a step, then accept the data from Unity.

        Args:
            count: the number of steps for executing Unity simulation.
        """
        RFUniverseBaseEnv.step(self, count)

    def step(self, action: gym.core.ActType) -> tuple[gym.core.ObsType, SupportsFloat, bool, bool, dict[str, Any]]:
        """
        Gym step.

        Args:
            action: gym action.
        """
        return gym.Env.step(self, action)

    def env_close(self):
        """
        Close the environment
        """
        RFUniverseBaseEnv.close(self)

    def close(self):
        """
        Close gym
        """
        gym.Env.close(self)
