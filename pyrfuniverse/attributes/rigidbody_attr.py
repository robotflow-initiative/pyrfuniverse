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
    """Add a constant force on a rigidbody. The rigidbody must be loaded into the scene and
    is distinguished by index.
    Args:
        Compulsory:
        id: The index of rigidbody, specified in returned message.
        force: A 3-d list inferring the force, in [x,y,z] order.
    """
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
    """Set the velocity of a rigidbody. The rigidbody must be loaded into the scene and
    is distinguished by index.
    Args:
        Compulsory:
        id: The index of rigidbody, specified in returned message.
        velocity: A 3-d float list inferring the velocity, in [x,y,z] order.
    """
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
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        self.data['velocity'] = [msg.read_float32() for _ in range(3)]
        self.data['angular_vel'] = [msg.read_float32() for _ in range(3)]
        return self.data

    def SetMass(self, mass: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetMass')
        msg.write_float32(mass)

        self.env.instance_channel.send_message(msg)

    def AddForce(self, force: list):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AddForce')
        msg.write_float32(force[0])
        msg.write_float32(force[1])
        msg.write_float32(force[2])

        self.env.instance_channel.send_message(msg)

    def SetVelocity(self, velocity: list):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetVelocity')
        msg.write_float32(velocity[0])
        msg.write_float32(velocity[1])
        msg.write_float32(velocity[2])

        self.env.instance_channel.send_message(msg)