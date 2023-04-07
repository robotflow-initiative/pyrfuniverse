from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv()

digit = env.InstanceObject(name='Digit', attr_type=attr.DigitAttr)
digit.SetTransform(position=[0, 0.015, 0])
target = env.InstanceObject(name='DigitTarget')
target.SetTransform(position=[0, 0.05, 0.015])
env.SetViewTransform(position=[-0.1, 0.033, 0.014], rotation=[0, 90, 0])
env.Pend()
env.close()

