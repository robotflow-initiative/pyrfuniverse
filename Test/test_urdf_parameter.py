from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
import os

env = RFUniverseBaseEnv()
robot = env.LoadURDF(path=os.path.abspath("../URDF/Franka/panda.urdf"), axis="z")
# robot = env.InstanceObject('franka_panda', attr_type=attr.ControllerAttr)
robot.SetTransform(position=[0, 0, 0])
robot.EnabledNativeIK(False)
env.ShowArticulationParameter(robot.id)

env.Pend()
