from pyrfuniverse.envs import RFUniverseBaseEnv


class RfuniverseRobotHubEnv(RFUniverseBaseEnv):
    def __init__(self, executable_file=None):
        super().__init__(
            executable_file=executable_file,
            camera_channel=True,
            articulation_channel=True,
            game_object_channel=True,
        )

    def step(self):
        # In each time step, this function must be called to make sure Unity works well.
        self._step()

    def reset(self):
        self.env.reset()
