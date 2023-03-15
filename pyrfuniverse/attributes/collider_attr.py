import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def GenerateVHACDColider(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GenerateVHACDColider')
    return msg


class ColliderAttr(attr.GameObjectAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        return self.data

    def GenerateVHACDColider(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GenerateVHACDColider')

        self.env.instance_channel.send_message(msg)