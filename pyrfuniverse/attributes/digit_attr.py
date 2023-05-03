import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import base64


class DigitAttr(attr.BaseAttr):
    """
    Class for simulating DIGIT tactile sensor.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['light']: Bytes of RGB light image in DIGIT.

            self.data['depth']: Bytes of depth image in DIGIT.
        """
        super().parse_message(msg)
        if msg.read_bool() is True:
            self.data['light'] = base64.b64decode(msg.read_string())
            self.data['depth'] = base64.b64decode(msg.read_string())
        return self.data


    def GetData(self):
        """
        Get data from DIGIT in RFUniverse.

        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetData')

        self.env.instance_channel.send_message(msg)