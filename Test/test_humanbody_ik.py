from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv(
    # executable_file='/home/yanbing/Project/rfuniverse/rfuniverse/Build/usr/local/RFUniverse/RFUniverse.x86_64',
    scene_file='HumanBodyIK.json'
)
env._step()

humanbody_id = 168242

for index in range(5):
    env.instance_channel.set_action(
        'HumanIKTargetDoMove',
        id=humanbody_id,
        index=index,
        position=[0, 0, 0.5],
        duration=1,
        speed_based=False,
        relative=True
        )
    env._step()
    while not env.instance_channel.data[humanbody_id]['move_done']:
        env._step()
    env.instance_channel.set_action(
        'HumanIKTargetDoMove',
        id=humanbody_id,
        index=index,
        position=[0, 0.5, 0],
        duration=1,
        speed_based=False,
        relative=True
    )
    env._step()
    while not env.instance_channel.data[humanbody_id]['move_done']:
        env._step()
    env.instance_channel.set_action(
        'HumanIKTargetDoMove',
        id=humanbody_id,
        index=index,
        position=[0, 0, -0.5],
        duration=1,
        speed_based=False,
        relative=True
    )
    env._step()
    while not env.instance_channel.data[humanbody_id]['move_done']:
        env._step()
    env.instance_channel.set_action(
        'HumanIKTargetDoMove',
        id=humanbody_id,
        index=index,
        position=[0, -0.5, 0],
        duration=1,
        speed_based=False,
        relative=True
    )
    env._step()
    while not env.instance_channel.data[humanbody_id]['move_done']:
        env._step()

while 1:
    env._step()