from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
import cv2
import numpy as np

env = RFUniverseBaseEnv(assets=['Camera', 'Rigidbody_Sphere_Draggable'])

camera = env.InstanceObject(name='Camera', attr_type=attr.CameraAttr)
camera.SetTransform(position=[-0.1, 0.033, 0.014], rotation=[0, 90, 0])
target = env.InstanceObject(name='Rigidbody_Sphere_Draggable')
target.SetTransform(position=[0, 0.05, 0.015])
env.step()
env.AlignCamera(camera.id)
env.SendLog("click End Pend button to start heat map record")
env.Pend()
camera.StartHeatMapRecord([target.id])
env.SendLog("click End Pend button to end heat map record")
env.Pend()
camera.EndHeatMapRecord()
camera.GetHeatMap()
env.step()
print(camera.data['heat_map'])
image_np = np.frombuffer(camera.data['heat_map'], dtype=np.uint8)
image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
print(image_np.shape)
env.close()
cv2.imshow("heatmap", image_np)
cv2.waitKey(0)
