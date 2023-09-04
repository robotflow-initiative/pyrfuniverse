from pyrfuniverse.envs.robotics import FrankaRoboticsEnv


class FrankaPickAndPlaceEnv(FrankaRoboticsEnv):
    def __init__(
        self,
        executable_file,
        max_episode_length,
        reward_type,
        asset_bundle_file=None,
        seed=None,
        assets: list = [],
    ):
        super().__init__(
            executable_file=executable_file,
            max_episode_length=max_episode_length,
            reward_type=reward_type,
            seed=seed,
            tolerance=0.05,
            load_object=True,
            target_in_air=True,
            block_gripper=False,
            target_xz_range=0.15,
            target_y_range=0.6,
            object_xz_range=0.15,
            asset_bundle_file=asset_bundle_file,
            assets=assets,
        )
