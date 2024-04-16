import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
from pyrfuniverse_test.extend.humanbody_attr import HumanbodyAttr

env = RFUniverseBaseEnv(scene_file="HumanBodyIK.json", ext_attr=[HumanbodyAttr])
env.step()
human = env.GetAttr(168242)
for index in range(5):
    human.HumanIKTargetDoMove(
        index=index, position=[0, 0, 0.5], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()
    human.HumanIKTargetDoMove(
        index=index, position=[0, 0.5, 0], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()
    human.HumanIKTargetDoMove(
        index=index, position=[0, 0, -0.5], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()
    human.HumanIKTargetDoMove(
        index=index, position=[0, -0.5, 0], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()

env.Pend()
env.close()
