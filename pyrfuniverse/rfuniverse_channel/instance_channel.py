import warnings

from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import RFUniverseChannel
import pyrfuniverse.attributes as attr

class InstanceChannel(RFUniverseChannel):

    def __init__(self, env, channel_id: str) -> None:
        super().__init__(channel_id)
        self.env = env
        self.data = {}

    def _parse_message(self, msg: IncomingMessage) -> None:
        # count = msg.read_int32()
        # for i in range(count):
            this_object_id = msg.read_int32()
            this_object_type = msg.read_string()

            # self.data[this_object_id] = eval('attr.' + this_object_type + '_attr.' + 'parse_message')(msg)

            attr_type = eval('attr.' + this_object_type)

            if this_object_id not in self.env.attrs:
                self.env.attrs[this_object_id] = attr_type(self.env, this_object_id)
            elif type(self.env.attrs[this_object_id]) != attr_type:
                self.env.attrs[this_object_id] = attr_type(self.env, this_object_id, self.env.attrs[this_object_id].data)

            self.data[this_object_id] = self.env.attrs[this_object_id].parse_message(msg)


    def set_action(self, action: str, attr_name=None, **kwargs) -> None:
        warnings.warn("set_action is deprecated, It will be removed in version 1.0", DeprecationWarning)
        """Set action and pass corresponding parameters
        Args:
            action: The action name.
            kwargs: keyword argument for action. The parameter list for each action is shown in each function.
        """
        try:
            if attr_name is not None:
                msg = eval('attr.' + attr_name + '.' + action)(kwargs)
                self.send_message(msg)
            else:
                for i in attr.__all__:
                    if eval('hasattr(attr.' + i + ',\'' + action + '\')'):
                        msg = eval('attr.' + i + '.' + action)(kwargs)
                        self.send_message(msg)
        except AttributeError:
            print('There is no action called \'%s\' or this function has bug, please fix it.' % action)
            exit(-1)



