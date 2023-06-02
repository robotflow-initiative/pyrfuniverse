import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
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

    def SetForceZoneParameters(self, orientation: float, intensity: float, turbulence: float, turbulence_frequency: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetForceZoneParameters')
        # Force zone orientation
        msg.write_float32(orientation)
        # Force zone intensity
        msg.write_float32(intensity)
        # Force zone turbulence
        msg.write_float32(turbulence)
        # Force zone turbulence frequency
        msg.write_float32(turbulence_frequency)

        self.env.instance_channel.send_message(msg)

    def SetSolverParameters(self, gravity: list):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetSolverParameters')
        # Solver gravity
        msg.write_float32(gravity[0])
        msg.write_float32(gravity[1])
        msg.write_float32(gravity[2])

        self.env.instance_channel.send_message(msg)