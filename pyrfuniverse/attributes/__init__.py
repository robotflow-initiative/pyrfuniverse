import pyrfuniverse.attributes.base_attr
import pyrfuniverse.attributes.camera_attr
import pyrfuniverse.attributes.activelightsensor_attr
import pyrfuniverse.attributes.collider_attr
import pyrfuniverse.attributes.controller_attr
import pyrfuniverse.attributes.gameobject_attr
import pyrfuniverse.attributes.rigidbody_attr
import pyrfuniverse.attributes.cloth_attr
import pyrfuniverse.attributes.humanbody_attr
import pyrfuniverse.attributes.graspsim_attr
import pyrfuniverse.attributes.custom_attr

# 新增脚本的命名规则必须是 *_attr,此处名称必须与Unity中新增attr的override的type一致
__all__ = [
    'base_attr', 'camera_attr', 'activelightsensor_attr', 'collider_attr', 'controller_attr', 'gameobject_attr', 'rigidbody_attr', 'cloth_attr', 'humanbody_attr', 'graspsim_attr', 'custom_attr'
]
