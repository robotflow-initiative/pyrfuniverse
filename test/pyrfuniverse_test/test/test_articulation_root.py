from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv()
shadow = env.InstanceObject("shadowhand", attr_type=attr.ControllerAttr)
shadow.SetPosition([0,1,0])
root = shadow.AddRoot6DOF()
env.step()

root.SetJointStiffness([100] * 6)
root.SetJointDamping([50] * 6)

#By setting the joint position of the root, the goal of moving the articulation body can be achieved.
root.SetIndexJointPosition(0, 0.5)
env.step(50)
root.SetIndexJointPosition(1, 0.5)
env.step(50)
root.SetIndexJointPosition(2, 0.5)
env.step(50)
root.SetIndexJointPosition(3, 45)
env.step(50)
root.SetIndexJointPosition(4, 45)
env.step(50)
root.SetIndexJointPosition(5, 45)
env.step(50)

env.Pend()
env.close()
