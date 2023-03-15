from pyrfuniverse.attributes.base_attr import BaseAttr
from pyrfuniverse.attributes.camera_attr import CameraAttr
from pyrfuniverse.attributes.gameobject_attr import GameObjectAttr
from pyrfuniverse.attributes.activelightsensor_attr import ActiveLightSensorAttr
from pyrfuniverse.attributes.collider_attr import ColliderAttr
from pyrfuniverse.attributes.controller_attr import ControllerAttr
from pyrfuniverse.attributes.rigidbody_attr import RigidbodyAttr
from pyrfuniverse.attributes.cloth_attr import ClothAttr
from pyrfuniverse.attributes.softbody_attr import SoftbodyAttr
from pyrfuniverse.attributes.humanbody_attr import HumanbodyAttr
from pyrfuniverse.attributes.graspsim_attr import GraspSimAttr
from pyrfuniverse.attributes.digit_attr import DigitAttr
from pyrfuniverse.attributes.pointcloud_attr import PointCloudAttr
from pyrfuniverse.attributes.custom_attr import CustomAttr

# 新增脚本的命名规则必须是 *_attr,此处名称必须与Unity中保持一致
__all__ = [
    'base_attr', 'camera_attr', 'activelightsensor_attr', 'collider_attr', 'controller_attr', 'gameobject_attr', 'rigidbody_attr', 'cloth_attr', 'softbody_attr', 'humanbody_attr', 'graspsim_attr', 'digit_attr', 'pointcloud_attr', 'custom_attr'
]
