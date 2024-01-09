import random
import os
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv()
env.DebugObjectPose()
env.EnabledGroundObiCollider(True)
mesh = env.LoadCloth(
    path=os.path.abspath("../Mesh/Tshirt.obj")
)
mesh.SetTransform(position=[0, 1, 0])
env.step(200)
mesh.GetParticles()
env.step()
position1 = mesh.data['particles'][500]
position2 = mesh.data['particles'][200]
point1 = env.InstanceObject("Empty")
point1.SetTransform(position=position1)
mesh.AddAttach(point1.id)
point2 = env.InstanceObject("Empty")
point2.SetTransform(position=position2)
mesh.AddAttach(point2.id)
env.step()

point1.DoMove([-0.25, 1, 0], 2, speed_based=False)
point2.DoMove([0.25, 1, 0], 2, speed_based=False)
point2.WaitDo()

while True:
    point1.DoMove([-0.25, 1, -0.5], 1)
    point2.DoMove([0.25, 1, -0.5], 1)
    point2.WaitDo()

    point1.DoMove([-0.25, 1, 0.5], 1)
    point2.DoMove([0.25, 1, 0.5], 1)
    point2.WaitDo()

env.Pend()
