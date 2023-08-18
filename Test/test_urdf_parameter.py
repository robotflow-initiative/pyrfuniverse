from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import os

env = RFUniverseBaseEnv()
kinova = env.LoadURDF(path=os.path.abspath('../URDF/Franka/panda.urdf'), axis='z')
kinova.SetTransform(position=[0, 0, 0])
env.ShowArticulationParameter(kinova.id)

env.Pend()
