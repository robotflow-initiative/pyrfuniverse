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
position = mesh.data['particles'][random.randint(0, len(mesh.data['particles'])-1)]
point = env.InstanceObject("Empty")
point.SetTransform(position=position)
mesh.AddAttach(point.id)
env.step()
point.DoMove([0, 1, 0], 0.2)
point.WaitDo()
point.DoMove([0.5, 1, 0], 0.5)
point.WaitDo()
point.DoMove([-0.5, 1, 0], 0.5)
point.WaitDo()
mesh.RemoveAttach(point.id)
env.Pend()
