import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility
import base64

def GetData(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetData')

    return msg


class DigitAttr(attr.BaseAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        if msg.read_bool() is True:
            self.data['light'] = base64.b64decode(msg.read_string())
            self.data['depth'] = base64.b64decode(msg.read_string())
        return self.data


    def GetData(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetData')

        self.env.instance_channel.send_message(msg)