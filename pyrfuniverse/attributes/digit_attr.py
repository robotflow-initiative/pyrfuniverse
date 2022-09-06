import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility
import base64

def parse_message(msg: IncomingMessage) -> dict:
    this_object_data = attr.base_attr.parse_message(msg)
    if msg.read_bool() is True:
        this_object_data['light'] = base64.b64decode(msg.read_string())
        this_object_data['depth'] = base64.b64decode(msg.read_string())
    return this_object_data

def GetData(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetData')

    return msg