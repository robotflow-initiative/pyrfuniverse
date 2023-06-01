from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
import pyrfuniverse.attributes.omplmanager_attr as rfu_ompl

env = RFUniverseBaseEnv(assets=['franka_panda', 'OmplManager'])

robot = env.InstanceObject(name='franka_panda', id=123456, attr_type=attr.ControllerAttr)
robot.EnabledNativeIK(False)
box1 = env.InstanceObject(name='Collider_Box', id=111111, attr_type=attr.ColliderAttr)
box1.SetTransform(position=[-0.3, 0, 0], scale=[0.2, 30, 0.2])
box2 = env.InstanceObject(name='Collider_Box', id=111112, attr_type=attr.ColliderAttr)
box2.SetTransform(position=[0.3, 0, 0], scale=[0.2, 30, 0.2])
env.step()

ompl_manager = env.InstanceObject(name='OmplManager', attr_type=attr.OmplManagerAttr)
ompl_manager.modify_robot(robot.id)
env.step()

start_state = [0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]
target_state = [ompl_manager.joint_upper_limit[j] * 0.9 for j in range(ompl_manager.joint_num)]

planner = rfu_ompl.RFUOMPL(ompl_manager, time_unit=5)

ompl_manager.set_state(start_state)
env.step(200)

ompl_manager.set_state(target_state)
env.step(200)

ompl_manager.set_state(start_state)
env.step(200)

is_sol, path = planner.plan_start_goal(start_state, target_state)

print(target_state)
print(path[-1])

while True:
    if is_sol:
        planner.execute(path)

# robot.SetIKTargetOffset(position=[0, 0.105, 0])
# env.step(200)
# gripper = env.GetAttr(1234560)
# gripper.GripperOpen()
# robot.IKTargetDoMove(position=[0, 0.7, 0.5], duration=0, speed_based=False)
# robot.IKTargetDoRotate(rotation=[0, 45, 180], duration=0, speed_based=False)
# env.step()
# robot.WaitDo()
#
# while 1:
#     box1 = env.InstanceObject(name='Rigidbody_Box', id=111111, attr_type=attr.RigidbodyAttr)
#     box1.SetTransform(position=[random.uniform(-0.5, -0.3), 0.03, random.uniform(0.3, 0.5)], scale=[0.06, 0.06, 0.06])
#     box2 = env.InstanceObject(name='Rigidbody_Box', id=222222, attr_type=attr.RigidbodyAttr)
#     box2.SetTransform(position=[random.uniform(0.3, 0.5), 0.03, random.uniform(0.3, 0.5)], scale=[0.06, 0.06, 0.06])
#     env.step(100)
#
#     position1 = box1.data['position']
#     position2 = box2.data['position']
#
#     robot.IKTargetDoMove(position=[position1[0], position1[1] + 0.5, position1[2]], duration=2, speed_based=False)
#     env.step()
#     robot.WaitDo()
#     robot.IKTargetDoMove(position=[position1[0], position1[1], position1[2]], duration=2, speed_based=False)
#     env.step()
#     robot.WaitDo()
#     gripper.GripperClose()
#     env.step(50)
#     robot.IKTargetDoMove(position=[0, 0.5, 0], duration=2, speed_based=False, relative=True)
#     env.step()
#     robot.WaitDo()
#     robot.IKTargetDoMove(position=[position2[0], position2[1] + 0.5, position2[2]], duration=4, speed_based=False)
#     env.step()
#     robot.WaitDo()
#     robot.IKTargetDoMove(position=[position2[0], position2[1] + 0.06, position2[2]], duration=2, speed_based=False)
#     env.step()
#     robot.WaitDo()
#     gripper.GripperOpen()
#     env.step(50)
#     robot.IKTargetDoMove(position=[0, 0.5, 0], duration=2, speed_based=False, relative=True)
#     env.step()
#     robot.WaitDo()
#     robot.IKTargetDoMove(position=[0, 0.7, 0.5], duration=2, speed_based=False)
#     env.step()
#     robot.WaitDo()
#     box1.Destroy()
#     box2.Destroy()
#     env.step()