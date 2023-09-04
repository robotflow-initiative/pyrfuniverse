from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.utils.rfuniverse_utility as utility

env = RFUniverseBaseEnv(scene_file="ArticulationIK.json")
ids = [639787, 985196, 221584, 8547820, 8547821]

for id in ids:
    current_robot = env.GetAttr(id)
    current_robot.IKTargetDoMove(position=[0, 0, -0.5], duration=0.1, relative=True)
    env.step()
    while not current_robot.data["move_done"]:
        env.step()
    current_robot.IKTargetDoMove(position=[0, -0.5, 0], duration=0.1, relative=True)
    env.step()
    while not current_robot.data["move_done"]:
        env.step()
    current_robot.IKTargetDoMove(position=[0, 0.5, 0.5], duration=0.1, relative=True)
    current_robot.IKTargetDoRotateQuaternion(
        quaternion=utility.UnityEularToQuaternion([90, 0, 0]),
        duration=30,
        relative=True,
    )
    env.step()
    while not current_robot.data["move_done"] or not current_robot.data["rotate_done"]:
        env.step()

env.Pend()
env.close()
