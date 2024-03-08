# test_custom_message

## 1 Basic Functions

- Send custom and dynamic messages

## 2 Implementation Process

### 2.1 Initialize Environment

```python
env = RFUniverseBaseEnv(assets=["CustomAttr"])
```

- Define a scene with `CustomAttr` assets

### 2.2 Send Custom Messages

```python
custom = env.InstanceObject(name="CustomAttr", id=123456, attr_type=attr.CustomAttr)
custom.CustomMessage(message="this is instance channel custom message")
env.step()
print(custom.data["custom_message"])
```

- Call the `InstanceObject` method to instantiate an object for sending custom messages

- Call the `CustomMessage` method to transmit the custom message you want to send

### 2.3 Send Dynamic Messages

```python
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
```

- Call the `AddListenerObject` method, where the first parameter is the message header, and the second parameter is the callback function to be invoked upon receiving a message

- Call the `SendObject` method, where the first parameter is the message header, and subsequent parameters are the contents of the message, supporting various types such as strings, boolean values, integers, floating-point numbers, and lists containing floating-point numbers, among others.