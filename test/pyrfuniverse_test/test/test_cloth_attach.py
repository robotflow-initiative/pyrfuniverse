import os
import sys

import cv2
import numpy as np
from pyrfuniverse.attributes.camera_attr import CameraAttr

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
camera = env.InstanceObject("Camera", attr_type=CameraAttr)
camera.SetTransform(position=[0, 1, -1])
camera.LookAt([0, 0.5, 0])
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

    camera.GetRGB(width=1024, height=512)
    env.step()
    image = np.frombuffer(camera.data["rgb"], dtype=np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    cv2.imshow("depth", image)
    cv2.waitKey(0)
