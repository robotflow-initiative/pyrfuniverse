from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv(scene_file='HumanBodyIK.json')
env.step()
human = env.GetAttr(168242)
for index in range(5):
    human.HumanIKTargetDoMove(index=index,
                              position=[0, 0, 0.5],
                              duration=1,
                              speed_based=False,
                              relative=True
                              )
    env.step()
    human.WaitDo()
    human.HumanIKTargetDoMove(index=index,
                              position=[0, 0.5, 0],
                              duration=1,
                              speed_based=False,
                              relative=True
                              )
    env.step()
    human.WaitDo()
    human.HumanIKTargetDoMove(index=index,
                              position=[0, 0, -0.5],
                              duration=1,
                              speed_based=False,
                              relative=True
                              )
    env.step()
    human.WaitDo()
    human.HumanIKTargetDoMove(index=index,
                              position=[0, -0.5, 0],
                              duration=1,
                              speed_based=False,
                              relative=True
                              )
    env.step()
    human.WaitDo()

while 1:
    env.step()
