import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def SetMass(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'mass']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetMass')
    msg.write_float32(kwargs['mass'])

    return msg

def AddForce(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'force']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('AddForce')
    msg.write_float32(kwargs['force'][0])
    msg.write_float32(kwargs['force'][1])
    msg.write_float32(kwargs['force'][2])

    return msg


def SetVelocity(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['index', 'velocity']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetVelocity')
    msg.write_float32(kwargs['velocity'][0])
    msg.write_float32(kwargs['velocity'][1])
    msg.write_float32(kwargs['velocity'][2])

    return msg

class RigidbodyAttr(attr.ColliderAttr):
    """
    Rigid body class.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['velocity']: The velocity of the object.

            self.data['angular_vel']: The angular velcity of the object.
        """
        super().parse_message(msg)
        self.data['velocity'] = [msg.read_float32() for _ in range(3)]
        self.data['angular_vel'] = [msg.read_float32() for _ in range(3)]
        return self.data

    def SetMass(self, mass: float):
        """
        Set the mass of this rigid body object

        Args:
            mass: Float, representing the mass of this rigid body.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetMass')
        msg.write_float32(mass)

        self.env.instance_channel.send_message(msg)

    def AddForce(self, force: list):
        """
        Add force to this rigid body object.

        Args:
            force: A list of length 3, representing the force added to this rigid body.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AddForce')
        msg.write_float32(force[0])
        msg.write_float32(force[1])
        msg.write_float32(force[2])

        self.env.instance_channel.send_message(msg)

    def SetVelocity(self, velocity: list):
        """
        Set the velocity of this rigid body object.

        Args:
            velocity: A list of length 3, representing the velocity of this rigid body.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetVelocity')
        msg.write_float32(velocity[0])
        msg.write_float32(velocity[1])
        msg.write_float32(velocity[2])

        self.env.instance_channel.send_message(msg)

    def SetKinematic(self, is_kinematic: bool):
        """
        Set the Rigidbody is kinematic or not.

        Args:
            is_kinematic: is kinematic or not.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetKinematic')
        msg.write_bool(is_kinematic)

        self.env.instance_channel.send_message(msg)