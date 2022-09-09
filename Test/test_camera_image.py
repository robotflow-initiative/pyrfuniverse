from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv()
env.asset_channel.set_action(
    'InstanceObject',
    name='Camera',
    id=123456
)
env.instance_channel.set_action(
    'SetTransform',
    id=123456,
    position=[0, 0.25, 0],
    rotation=[30, 0, 0],
)
env.instance_channel.set_action(
    'GetDepth',
    id=123456,
    width=512,
    height=512,
    zero_dis=1,
    one_dis=5
)
env.instance_channel.set_action(
    'GetRGB',
    id=123456,
    width=512,
    height=512
)
env._step()
print(env.instance_channel.data[123456]['rgb'])
print(env.instance_channel.data[123456]['depth'])
while 1:
    env._step()
