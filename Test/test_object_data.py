from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv(assets=['Rigidbody_Box', 'franka_panda'])

box = env.InstanceObject(name='Rigidbody_Box', id=123456, attr_type=attr.RigidbodyAttr)
box.SetTransform(position=[0, 1, 0])
env.step(5)
print(box.data['position'])
print(box.data['rotation'])
print(box.data['quaternion'])
print(box.data['local_position'])
print(box.data['local_rotation'])
print(box.data['local_quaternion'])

print(box.data['velocity'])
print(box.data['angular_vel'])

robot = env.InstanceObject(name='franka_panda', id=789789, attr_type=attr.ControllerAttr)
robot.SetTransform(position=[1, 0, 0])
env.step()
print(robot.data['number_of_joints'])
print(robot.data['positions'])
print(robot.data['rotations'])
print(robot.data['quaternion'])
print(robot.data['local_positions'])
print(robot.data['local_rotations'])
print(robot.data['local_quaternion'])
print(robot.data['velocities'])
print(robot.data['number_of_moveable_joints'])
print(robot.data['joint_positions'])
print(robot.data['joint_velocities'])

env.close()
