from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)

env = RFUniverseBaseEnv(assets=["CustomAttr"])

# custom message
custom = env.InstanceObject(name="CustomAttr", id=123456, attr_type=attr.CustomAttr)
custom.CustomMessage(message="this is instance channel custom message")
env.step()
print(custom.data["custom_message"])


# dynamic message
def dynamic_message_callback(msg: IncomingMessage):
    print(msg.read_string())
    print(msg.read_string())
    print(msg.read_string())
    print(msg.read_int32())
    print(msg.read_string())
    print(msg.read_float32())
    print(msg.read_string())
    print(msg.read_bool())
    print(msg.read_string())
    print(msg.read_float32_list())


env.AddListener("DynamicMessage", dynamic_message_callback)
env.SendMessage(
    "DynamicMessage",
    "string:",
    "this is dynamic message",
    "int:",
    123456,
    "bool:",
    True,
    "float:",
    4849.6564,
    "list:",
    [616445.085, 9489984.0, 65419596.0, 9849849.0],
)
env.step()


# dynamic object
def dynamic_object_callback(args):
    print(args[0])
    print(args[1])
    print(args[2])
    print(args[3])
    print(args[4])
    print(args[5])
    print(args[6])
    print(args[7])
    print(args[8])
    print(args[9])
    print(args[10])
    print(args[11])
    print(args[12])
    print(args[13])


env.AddListenerObject("DynamicObject", dynamic_object_callback)
env.SendObject(
    "DynamicObject",
    "string:",
    "this is dynamic object",
    "int:",
    123456,
    "bool:",
    True,
    "float:",
    4849.6564,
    "list:",
    [616445.085, 9489984.0, 65419596.0, 9849849.0],
    "dict:",
    {"1": 1, "2": 2, "3": 3},
    "tuple:",
    ("1", 1, 0.562),
)
env.step()

env.Pend()
env.close()
