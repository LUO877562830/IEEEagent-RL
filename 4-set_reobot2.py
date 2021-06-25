#机器人控制与相机追踪
import pybullet as p
import pybullet_data
from time import sleep
import time
from pprint import pprint

use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)

else:
    serve_id = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

#创建过程中不渲染、不展示套件、禁用 tinyrenderer
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setGravity(0, 0, -9.8)

plant = p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf")

available_joints_indexes = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED]
pprint([p.getJointInfo(robot, i)[1] for i in available_joints_indexes])
wheel_joints_indexes = [i for i in available_joints_indexes if "wheel" in str(p.getJointInfo(robot, i)[1])]

#渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setRealTimeSimulation(0)

target_v = 10
max_force = 10

'''
##force设为0来禁用一些关节马达
maxForce = 0
mode = p.VELOCITY_CONTROL
p.setJointMotorControl2(objUid, jointIndex,
 	controlMode=mode, force=maxForce)
'''


for i in range(1000):
    p.stepSimulation()
    p.setJointMotorControlArray(
        bodyUniqueId = robot,
        jointIndices = wheel_joints_indexes,
        controlMode = p.VELOCITY_CONTROL,
        targetVelocities = [target_v for _ in wheel_joints_indexes],
        forces = [max_force for _ in wheel_joints_indexes]
    )

    location, _ = p.getBasePositionAndOrientation(robot)
    p.resetDebugVisualizerCamera(
        cameraDistance = 3,
        cameraYaw = 110,
        cameraPitch = -30,
        cameraTargetPosition = location
    )

    time.sleep(1. / 240.)

p.disconnect(serve_id)





