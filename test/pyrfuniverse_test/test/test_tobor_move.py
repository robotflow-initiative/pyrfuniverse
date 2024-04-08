from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv(assets=["tobor_r300_ag95_ag95"])

torbor = env.InstanceObject(name="tobor_r300_ag95_ag95", attr_type=attr.ControllerAttr)
torbor.SetTransform(position=[0, 0.05, 0])
torbor.SetImmovable(False)
env.step()
while 1:
    torbor.MoveForward(1, 0.2)
    env.step(300)
    torbor.TurnLeft(90, 30)
    env.step(300)
    torbor.MoveForward(1, 0.2)
    env.step(300)
    torbor.TurnLeft(90, 30)
    env.step(300)
    torbor.MoveForward(1, 0.2)
    env.step(300)
    torbor.TurnRight(90, 30)
    env.step(300)
    torbor.MoveBack(1, 0.2)
    env.step(300)
    torbor.TurnRight(90, 30)
    env.step(300)
