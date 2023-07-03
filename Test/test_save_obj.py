import os
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv(scene_file='SimpleYCBModel.json')

model = []

for i in env.attrs:
    if type(env.attrs[i]) is attr.RigidbodyAttr:
        model.append(i)

env.ExportOBJ(model, os.path.abspath('../Mesh/scene_mesh.obj'))

env.Pend()
env.close()

