from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from pyrfuniverse_test import urdf_path

env = RFUniverseBaseEnv()
robot = env.LoadURDF(path=os.path.join(urdf_path, "Franka/panda.urdf"), axis="z")
# robot = env.InstanceObject('franka_panda', attr_type=attr.ControllerAttr)
robot.SetTransform(position=[0, 0, 0])
robot.EnabledNativeIK(False)
env.ShowArticulationParameter(robot.id)

env.Pend()
