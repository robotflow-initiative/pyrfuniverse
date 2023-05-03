import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility

def Translate(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'translation']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('Translate')
    for i in range(3):
        msg.write_float32(kwargs['translation'][i])

    return msg


def Rotate(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'rotation']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('Rotate')
    for i in range(3):
        msg.write_float32(kwargs['rotation'][i])

    return msg


def SetColor(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'color']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetColor')
    for i in range(4):
        msg.write_float32(kwargs['color'][i])

    return msg


class GameObjectAttr(attr.BaseAttr):
    """
    Basic game object attribute class.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['3d_bounding_box']: The 3d bounding box of objects.
        """
        super().parse_message(msg)
        if msg.read_bool():
            self.data['3d_bounding_box'] = {}
            self.data['3d_bounding_box']['position'] = [msg.read_float32() for _ in range(3)]
            self.data['3d_bounding_box']['rotation'] = [msg.read_float32() for _ in range(3)]
            self.data['3d_bounding_box']['size'] = [msg.read_float32() for _ in range(3)]
        return self.data

    def SetColor(self, color: list):
        """
        Set object color.

        Args:
            color: A list of length 4, represenging r, g, b and a. Each float is in range (0, 1).
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetColor')
        for i in range(4):
            msg.write_float32(color[i])

        self.env.instance_channel.send_message(msg)

    def EnabledRender(self, enabled: bool):
        """
        Enable or disable rendering system.

        Args:
            enabled: Bool, Ture for enable rendering and False for disable rendering.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('EnabledRender')
        msg.write_bool(enabled)

        self.env.instance_channel.send_message(msg)

    def SetTexture(self, path: str):
        """
        Set the texture of object.

        Args:
            path: Str, the absolute path for texture file.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetTexture')
        msg.write_string(path)

        self.env.instance_channel.send_message(msg)

    def Get3DBBox(self):
        """
        Get the 3d bounding box of this object.
        
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Get3DBBox')

        self.env.instance_channel.send_message(msg)



