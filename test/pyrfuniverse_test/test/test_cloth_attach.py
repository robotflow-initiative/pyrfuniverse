import os
import sys

import cv2
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import os.path
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
from pyrfuniverse_test import mesh_path

env = RFUniverseBaseEnv()
env.DebugObjectPose()
env.EnabledGroundObiCollider(True)
t_shirt_path = os.path.join(mesh_path, 'Tshirt.obj')
mesh = env.LoadCloth(
    path=t_shirt_path
)
mesh.SetTransform(position=[0, 0.5, 0])
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
    point1.DoMove([-0.25, 1, -0.5], 0.5)
    point2.DoMove([0.25, 1, -0.5], 0.5)
    point2.WaitDo()

    point1.DoMove([-0.25, 1, 0.5], 0.5)
    point2.DoMove([0.25, 1, 0.5], 0.5)
    point2.WaitDo()
