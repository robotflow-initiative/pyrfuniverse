from pyrfuniverse.envs.robotics import FrankaRoboticsEnv


class FrankaReachEnv(FrankaRoboticsEnv):
    def __init__(
        self,
        max_episode_length,
        reward_type,
        executable_file=None,
        scene_file=None,
        asset_bundle_file=None,
        seed=None,
        assets: list = [],
    ):
        super().__init__(
            max_episode_length=max_episode_length,
            reward_type=reward_type,
            tolerance=0.05,
            load_object=False,
            target_in_air=True,
            block_gripper=True,
            target_xz_range=0.15,
            target_y_range=0.6,
            object_xz_range=0.15,
            seed=seed,
            executable_file=executable_file,
            scene_file=scene_file,
            asset_bundle_file=None,
            assets=assets,
        )
