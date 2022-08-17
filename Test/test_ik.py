from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.utils.rfuniverse_utility as tool

env = RFUniverseBaseEnv(scene_file='IK.json')
env._step()
ids = [639787, 985196, 221584, 8547820, 8547821]

for id in ids:
    env.instance_channel.set_action(
        'IKTargetDoMove',
        id=id,
        position=[0, 0, -0.5],
        speed=0.1,
        relative=True
        )
    env._step()
    while not env.instance_channel.data[id]['move_done']:
        env._step()
    env.instance_channel.set_action(
        'IKTargetDoMove',
        id=id,
        position=[0, -0.5, 0],
        speed=0.1,
        relative=True
        )
    env._step()
    while not env.instance_channel.data[id]['move_done']:
        env._step()
    while not env.instance_channel.data[id]['move_done']:
        env._step()
    env.instance_channel.set_action(
        'IKTargetDoMove',
        id=id,
        position=[0, 0.5, 0.5],
        speed=0.1,
        relative=True
        )
    env.instance_channel.set_action(
        'IKTargetDoRotateQuaternion',
        id=id,
        quaternion=tool.UnityEularToQuaternion([90, 0, 0]),
        speed=30,
        relative=True
        )
    env._step()
    while not env.instance_channel.data[id]['move_done'] or not env.instance_channel.data[id]['rotate_done']:
        env._step()

while 1:
    env._step()