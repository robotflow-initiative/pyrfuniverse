import os.path
import pyrfuniverse.attributes as attr
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv()
env.SetViewBackGround([0.0, 0.0, 0.0])
point_cloud = env.InstanceObject(
    name="PointCloud", id=123456, attr_type=attr.PointCloudAttr
)
point_cloud.ShowPointCloud(ply_path=os.path.abspath("../Mesh/000000_000673513312.ply"))
point_cloud.SetTransform(rotation=[-90, 0, 0])
point_cloud.SetRadius(radius=0.001)

env.Pend()
env.close()
