from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv(
    executable_file='/home/yanbing/Project/rfuniverse/rfuniverse/Build/usr/local/RFUniverse/RFUniverse.x86_64',
    assets=['CustomAttr']
)

# asset_channel custom message
env.asset_channel.set_action(
    'CustomMessage',
    message='this is a asset channel custom message'
)
env._step()
msg = env.asset_channel.data['custom_message']
print(msg)


# instance_channel custom message
env.asset_channel.set_action(
    'InstanceObject',
    name='CustomAttr',
    id=123456
)
env.instance_channel.set_action(
    'CustomMessage',
    id=123456,
    message='this is a instance channel custom message'
)
env._step()
msg = env.instance_channel.data[123456]['custom_message']
print(msg)

while 1:
    env._step()