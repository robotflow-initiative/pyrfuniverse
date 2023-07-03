from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import os

env = RFUniverseBaseEnv()

ur5 = env.LoadURDF(path=os.path.abspath('../URDF/01b24b02-0e4e-11ed-81d4-ec2e98c7e246/motion_unity.urdf'), native_ik=False)
ur5.SetTransform(position=[0, 1, 0])

env.Pend()
