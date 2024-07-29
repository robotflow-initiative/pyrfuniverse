from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv(assets=["tobor_r300_ag95_ag95"])

tobor = env.InstanceObject(name="tobor_r300_ag95_ag95", attr_type=attr.ControllerAttr)
tobor.SetTransform(position=[0, 0.05, 0])
tobor.SetImmovable(False)
env.step()
while 1:
    tobor.MoveForward(1, 0.2)
    env.step(300)
    tobor.TurnLeft(90, 30)
    env.step(300)
    tobor.MoveForward(1, 0.2)
    env.step(300)
    tobor.TurnLeft(90, 30)
    env.step(300)
    tobor.MoveForward(1, 0.2)
    env.step(300)
    tobor.TurnRight(90, 30)
    env.step(300)
    tobor.MoveBack(1, 0.2)
    env.step(300)
    tobor.TurnRight(90, 30)
    env.step(300)
