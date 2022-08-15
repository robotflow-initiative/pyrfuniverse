from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import RFUniverseChannel
import base64

class InstanceChannel(RFUniverseChannel):

    def __init__(self, channel_id: str, env) -> None:
        super().__init__(channel_id)
        self.env = env
        self.data = {}

    def _parse_message(self, msg: IncomingMessage) -> None:
        title = msg.read_string()
        assert title == 'Instance Info', \
            'The information %s is not for game_object, please check uuid to avoid repeat.' % title
        count = msg.read_int32()

        for i in range(count):
            this_object_id = msg.read_int32()
            this_object_data = {}
            this_object_data['name'] = msg.read_string()
            this_object_data['type'] = msg.read_string()
            this_object_data['position'] = [msg.read_float32() for _ in range(3)]
            this_object_data['rotation'] = [msg.read_float32() for _ in range(3)]
            this_object_data['quaternion'] = [msg.read_float32() for _ in range(4)]
            if msg.read_bool() is True:
                this_object_data['result_local_point'] = msg.read_float32_list()
            if msg.read_bool() is True:
                this_object_data['result_world_point'] = msg.read_float32_list()

            #if this_object_data['type'] == 'GameObject':
            if this_object_data['type'] == 'Rigidbody':
                this_object_data['velocity'] = [msg.read_float32() for i in range(3)]
                this_object_data['angular_vel'] = [msg.read_float32() for i in range(3)]
            elif this_object_data['type'] == 'Controller':
                this_object_data['number_of_joints'] = msg.read_int32()
                # Position
                this_object_data['positions'] = self._parse_raw_list_3(msg.read_float32_list())
                # RotationEuler
                this_object_data['rotations'] = self._parse_raw_list_3(msg.read_float32_list())
                # RotationQuaternion
                this_object_data['quaternion'] = self._parse_raw_list_4(msg.read_float32_list())
                # Velocity
                this_object_data['velocities'] = self._parse_raw_list_3(msg.read_float32_list())
                # Each joint position
                this_object_data['joint_positions'] = msg.read_float32_list()
                # Each joint velocity
                this_object_data['joint_velocities'] = msg.read_float32_list()
                # Whether all parts are stable
                this_object_data['all_stable'] = msg.read_bool()
                if msg.read_bool() is True:
                    this_object_data['gravity_forces'] = msg.read_float32_list()
                    this_object_data['coriolis_centrifugal_forces'] = msg.read_float32_list()
                    this_object_data['drive_forces'] = msg.read_float32_list()
            elif this_object_data['type'] == 'Camera':
                this_object_data['near_plane'] = msg.read_float32()
                this_object_data['far_plane'] = msg.read_float32()
                this_object_data['FOV'] = msg.read_float32()
                this_object_data['target_display'] = msg.read_int32()
                this_object_data['width'] = msg.read_int32()
                this_object_data['height'] = msg.read_int32()
                if msg.read_bool() is True:
                    this_object_data['rgb'] = base64.b64decode(msg.read_string())
                if msg.read_bool() is True:
                    this_object_data['normal'] = base64.b64decode(msg.read_string())
                if msg.read_bool() is True:
                    this_object_data['id'] = base64.b64decode(msg.read_string())
                if msg.read_bool() is True:
                    this_object_data['depth'] = base64.b64decode(msg.read_string())
                if msg.read_bool() is True:
                    this_object_data['depth_exr'] = base64.b64decode(msg.read_string())
            #elif this_object_data['type'] == 'Cloth':
            #elif this_object_data['type'] == 'ClothWithGrasping':
            elif this_object_data['type'] == 'Softbody':
                # Number of particles
                this_object_data['number_of_particles'] = msg.read_int32()
                # Average Positions
                this_object_data['position'] = [msg.read_float32() for i in range(3)]
                this_object_data['orientation'] = [msg.read_float32() for i in range(4)]
                this_object_data['velocity'] = [msg.read_float32() for i in range(3)]
                this_object_data['angular_vel'] = [msg.read_float32() for i in range(3)]
            elif this_object_data['type'] == 'HumanDressing':
                # Grasp point position
                this_object_data['grasp_position'] = [msg.read_float32() for i in range(3)]
                # Grasp point rotation
                this_object_data['grasp_rotation'] = [msg.read_float32() for i in range(3)]
                # Grasp point velocity
                this_object_data['grasp_velocity'] = [msg.read_float32() for i in range(3)]
                # Grasp point angular velocity
                this_object_data['grasp_angular_vel'] = [msg.read_float32() for i in range(3)]
                # Target position
                this_object_data['target_position'] = [msg.read_float32() for i in range(3)]
                # Target rotation
                this_object_data['target_rotation'] = [msg.read_float32() for i in range(3)]

            self.data[this_object_id] = this_object_data

    def _parse_raw_list_3(self, raw_list):
        length = len(raw_list)
        assert length % 3 == 0
        number_of_parts = length // 3
        norm_list = []
        for j in range(number_of_parts):
            transform = [raw_list[3 * j], raw_list[3 * j + 1], raw_list[3 * j + 2]]
            norm_list.append(transform)

        return norm_list

    def _parse_raw_list_4(self, raw_list):
        length = len(raw_list)
        assert length % 4 == 0
        number_of_parts = length // 4
        norm_list = []
        for j in range(number_of_parts):
            transform = [raw_list[4 * j], raw_list[4 * j + 1], raw_list[4 * j + 2], raw_list[4 * j + 3]]
            norm_list.append(transform)

        return norm_list

    def SetTransform(self, kwargs: dict) -> None:
        """Set the transform of a object, specified by id.
        Args:
            Compulsory:
            id: The id of object.

            Optional:
            position: A 3-d list inferring object's position, in [x,y,z] order.
            rotation: A 3-d list inferring object's rotation, in [x,y,z] order.
            scale: A 3-d list inferring object's rotation, in [x,y,z] order.
        """
        compulsory_params = ['id']
        optional_params = ['position', 'rotation', 'scale']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        position = None
        set_position = False
        rotation = None
        set_rotation = False
        scale = None
        set_scale = False

        if optional_params[0] in kwargs.keys(): # position
            position = kwargs[optional_params[0]]
            set_position = True
            assert type(position) == list and len(position) == 3, \
                'Argument position must be a 3-d list.'

        if optional_params[1] in kwargs.keys(): # rotation
            rotation = kwargs[optional_params[1]]
            set_rotation = True
            assert type(rotation) == list and len(rotation) == 3, \
                'Argument rotation must be a 3-d list.'

        if optional_params[2] in kwargs.keys(): # scale
            scale = kwargs[optional_params[2]]
            set_scale = True
            assert type(scale) == list and len(scale) == 3, \
                'Argument rotation must be a 3-d list.'

        msg.write_int32(kwargs['id'])
        msg.write_string('SetTransform')
        msg.write_bool(set_position)
        msg.write_bool(set_rotation)
        msg.write_bool(set_scale)

        if set_position:
            for i in range(3):
                msg.write_float32(position[i])

        if set_rotation:
            for i in range(3):
                msg.write_float32(rotation[i])

        if set_scale:
            for i in range(3):
                msg.write_float32(scale[i])

        self.send_message(msg)

    def SetRotationQuaternion(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'quaternion']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        msg.write_int32(kwargs['id'])
        msg.write_string('SetRotationQuaternion')
        for i in range(4):
            msg.write_float32(kwargs['quaternion'][i])
        self.send_message(msg)

    def SetActive(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'active']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('SetActive')
        msg.write_bool(kwargs['active'])

        self.send_message(msg)

    def SetParent(self, kwargs: dict) -> None:
        """Set parent of a object inferred by the id
        Args:
            Compulsory:
            id: The id of object, specified in returned message.
            parent_id: The id of parent object
            parent_name: The name of parent object
        """
        compulsory_params = ['id', 'parent_id', 'parent_name']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('SetParent')
        msg.write_int32(kwargs['parent_id'])
        msg.write_string(kwargs['parent_name'])

        self.send_message(msg)

    def SetLayer(self, kwargs: dict) -> None:
        """Set layer of a object inferred by the id
        Args:
            Compulsory:
            id: The id of object, specified in returned message.
            layer: The layer of object
        """
        compulsory_params = ['id', 'layer']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('SetLayer')
        msg.write_int32(kwargs['layer'])

        self.send_message(msg)


    def Destroy(self, kwargs: dict) -> None:
        """Destroy a object inferred by the id
        Args:
            Compulsory:
            id: The id of object, specified in returned message.
        """
        compulsory_params = ['id']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('Destroy')

        self.send_message(msg)

    def GetLoaclPointFromWorld(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'point']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('GetLoaclPointFromWorld')
        msg.write_float32(kwargs['point'][0])
        msg.write_float32(kwargs['point'][1])
        msg.write_float32(kwargs['point'][2])
        self.send_message(msg)

    def GetWorldPointFromLocal(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'point']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('GetWorldPointFromLocal')
        msg.write_float32(kwargs['point'][0])
        msg.write_float32(kwargs['point'][1])
        msg.write_float32(kwargs['point'][2])
        self.send_message(msg)

    # GameObject
    def Translate(self, kwargs: dict) -> None:
        """Translate a game object by a given distance, in meter format. Note that this command will translate the
           object relative to the current position.
        Args:
            Compulsory:
            index: The index of object, specified in returned message.
            translation: A 3-d list inferring the relative translation, in [x,y,z] order.
        """
        compulsory_params = ['id', 'translation']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('Translate')
        for i in range(3):
            msg.write_float32(kwargs['translation'][i])

        self.send_message(msg)

    def Rotate(self, kwargs: dict) -> None:
        """Rotate a game object by a given rotation, in euler angle format. Note that this command will rotate the
           object relative to the current state. The rotation order will be z axis first, x axis next, and z axis last.
        Args:
            Compulsory:
            index: The index of object, specified in returned message.
            rotation: A 3-d list inferring the relative rotation, in [x,y,z] order.
        """
        compulsory_params = ['id', 'rotation']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('Rotate')
        for i in range(3):
            msg.write_float32(kwargs['rotation'][i])

        self.send_message(msg)

    def SetColor(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'color']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('SetColor')
        for i in range(4):
            msg.write_float32(kwargs['color'][i])

        self.send_message(msg)

    # Controller
    def SetJointPosition(self, kwargs: dict) -> None:
        """Set the target positions for each joint in a specified articulation body.
        Args:
            Compulsory:
            index: The index of articulation body, specified in returned message.
            joint_positions: A list inferring each joint's position in the specified acticulation body.

            Optional:
            speed_scales: A list inferring each joint's speed scale. The length must be the same with joint_positions.
        """
        compulsory_params = ['id', 'joint_positions']
        optional_params = ['speed_scales']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        joint_positions = kwargs['joint_positions']
        num_joints = len(joint_positions)

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointPosition')
        msg.write_int32(num_joints)
        msg.write_float32_list(kwargs['joint_positions'])
        if 'speed_scales' in kwargs.keys():
            assert num_joints == len(kwargs['speed_scales']), \
                'The length of joint_positions and speed_scales are not equal.'
            msg.write_float32_list(kwargs['speed_scales'])
        else:
            msg.write_float32_list([1.0 for i in range(num_joints)])

        self.send_message(msg)

    def SetJointPositionDirectly(self, kwargs: dict) -> None:
        """Set the target positions for each joint in a specified articulation body. Note that this function will move
           all joints directrly to its target joint position, and ignoring the physical effects during moving.
        Args:
            Compulsory:
            index: The index of articulation body, specified in returned message.
            joint_positions: A list inferring each joint's position in the specified acticulation body.
        """
        compulsory_params = ['id', 'joint_positions']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        joint_positions = kwargs['joint_positions']
        num_joints = len(joint_positions)

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointPositionDirectly')
        msg.write_int32(num_joints)
        msg.write_float32_list(kwargs['joint_positions'])

        self.send_message(msg)

    def SetJointPositionContinue(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'interval', 'time_joint_positions']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        time_joint_positions = kwargs['time_joint_positions']
        num_times = len(time_joint_positions)
        num_joints = len(time_joint_positions[0])
        interval = kwargs['interval']

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointPositionContinue')
        msg.write_int32(num_times)
        msg.write_int32(num_joints)
        msg.write_int32(interval)
        for i in range(num_times):
            msg.write_float32_list(time_joint_positions[i])

        self.send_message(msg)

    def SetJointVelocity(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'joint_velocitys']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        joint_velocitys = kwargs['joint_velocitys']
        num_joints = len(joint_velocitys)

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointVelocity')
        msg.write_int32(num_joints)
        msg.write_float32_list(kwargs['joint_velocitys'])

        self.send_message(msg)

    def SetJointForce(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'joint_forces']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        joint_positions = kwargs['joint_positions']
        num_joints = len(joint_positions)

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointForce')
        msg.write_int32(num_joints)
        for i in range(num_joints):
            msg.write_float32(joint_positions[i][0])
            msg.write_float32(joint_positions[i][1])
            msg.write_float32(joint_positions[i][2])

        self.send_message(msg)

    def SetJointForceAtPosition(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'joint_forces', 'forces_position']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        joint_positions = kwargs['joint_forces']
        forces_position = kwargs['forces_position']
        num_joints = len(joint_positions)

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointForceAtPosition')
        msg.write_int32(num_joints)
        for i in range(num_joints):
            msg.write_float32(joint_positions[i][0])
            msg.write_float32(joint_positions[i][1])
            msg.write_float32(joint_positions[i][2])
            msg.write_float32(forces_position[i][0])
            msg.write_float32(forces_position[i][1])
            msg.write_float32(forces_position[i][2])

        self.send_message(msg)

    def SetJointTorque(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'joint_torque']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        joint_torque = kwargs['joint_torque']
        num_joints = len(joint_torque)

        msg.write_int32(kwargs['id'])
        msg.write_string('SetJointTorque')
        msg.write_int32(num_joints)
        for i in range(num_joints):
            msg.write_float32(joint_torque[i][0])
            msg.write_float32(joint_torque[i][1])
            msg.write_float32(joint_torque[i][2])

        self.send_message(msg)

    # only work on unity 2022.1+
    def GetJointInverseDynamicsForce(self, kwargs: dict) -> None:
        compulsory_params = ['id']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        msg.write_int32(kwargs['id'])
        msg.write_string('GetJointInverseDynamicsForce')
        self.send_message(msg)

    def SetImmovable(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'immovable']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        msg.write_int32(kwargs['id'])
        msg.write_string('SetImmovable')
        msg.write_bool(kwargs['immovable'])
        self.send_message(msg)

    def EnabledNativeIK(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'enabled']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        msg.write_int32(kwargs['id'])
        msg.write_string('EnabledNativeIK')
        msg.write_bool(kwargs['enabled'])
        self.send_message(msg)

    # Rigidbody
    def AddForce(self, kwargs: dict):
        """Add a constant force on a rigidbody. The rigidbody must be loaded into the scene and
        is distinguished by index.
        Args:
            Compulsory:
            id: The index of rigidbody, specified in returned message.
            force: A 3-d list inferring the force, in [x,y,z] order.
        """
        compulsory_params = ['id', 'force']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('AddForce')
        msg.write_float32(kwargs['force'][0])
        msg.write_float32(kwargs['force'][1])
        msg.write_float32(kwargs['force'][2])

        self.send_message(msg)

    def SetVelocity(self, kwargs: dict):
        """Set the velocity of a rigidbody. The rigidbody must be loaded into the scene and
        is distinguished by index.
        Args:
            Compulsory:
            id: The index of rigidbody, specified in returned message.
            velocity: A 3-d float list inferring the velocity, in [x,y,z] order.
        """
        compulsory_params = ['index', 'velocity']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('SetVelocity')
        msg.write_float32(kwargs['velocity'][0])
        msg.write_float32(kwargs['velocity'][1])
        msg.write_float32(kwargs['velocity'][2])

        self.send_message(msg)

    # camera
    def GetRGB(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'width', 'height']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('GetRGB')
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])

        self.send_message(msg)

    def GetNormal(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'width', 'height']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('GetNormal')
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])

        self.send_message(msg)

    def GetDepth(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'width', 'height', 'zero_dis', 'one_dis']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('GetDepth')
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        msg.write_int32(kwargs['zero_dis'])
        msg.write_int32(kwargs['one_dis'])

        self.send_message(msg)

    def GetDepthEXR(self, kwargs: dict) -> None:
        compulsory_params = ['id', 'width', 'height']
        optional_params = []
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()

        msg.write_int32(kwargs['id'])
        msg.write_string('GetDepthEXR')
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])

        self.send_message(msg)


    # human_dressing
    def SetTargetX(self, kwargs: dict) -> None:
        compulsory_params = ['targetx']
        self._check_kwargs(kwargs, compulsory_params)

        msg = OutgoingMessage()
        msg.write_string('SetTargetX')
        msg.write_float32(kwargs['targetx'])

        self.send_message(msg)
