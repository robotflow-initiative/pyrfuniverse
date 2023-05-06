import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def SetForceZoneParameters(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = ['orientation', 'intensity', 'turbulence', 'turbulence_frequency']
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetForceZoneParameters')

    # Force zone orientation
    if 'orientation' in kwargs.keys():
        msg.write_bool(True)
        msg.write_float32(kwargs['orientation'])
    else:
        msg.write_bool(False)

    # Force zone intensity
    if 'intensity' in kwargs.keys():
        msg.write_bool(True)
        msg.write_float32(kwargs['intensity'])
    else:
        msg.write_bool(False)

    # Force zone turbulence
    if 'turbulence' in kwargs.keys():
        msg.write_bool(True)
        msg.write_float32(kwargs['turbulence'])
    else:
        msg.write_bool(False)

    # Force zone turbulence frequency
    if 'turbulence_frequency' in kwargs.keys():
        msg.write_bool(True)
        msg.write_float32(kwargs['turbulence_frequency'])
    else:
        msg.write_bool(False)

    return msg


def SetSolverParameters(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = ['gravity', ]
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetSolverParameters')

    # Solver gravity
    if 'gravity' in kwargs.keys():
        msg.write_bool(True)
        msg.write_float32(kwargs['gravity'][0])
        msg.write_float32(kwargs['gravity'][1])
        msg.write_float32(kwargs['gravity'][2])
    else:
        msg.write_bool(False)

    return msg


class FallingClothAttr(attr.BaseAttr):

    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        self.data['num_particles'] = msg.read_int32()
        self.data['avg_position'] = [msg.read_float32() for _ in range(3)]
        self.data['avg_velocity'] = [msg.read_float32() for _ in range(3)]
        self.data['force_zone_orientation'] = msg.read_float32()
        self.data['force_zone_intensity'] = msg.read_float32()
        self.data['force_zone_turbulence'] = msg.read_float32()
        self.data['force_zone_turbulence_frequency'] = msg.read_float32()
        return self.data

