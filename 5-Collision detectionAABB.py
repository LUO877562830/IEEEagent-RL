#碰撞检测与激光模拟
import pybullet as p
import pybullet_data
import time
from time import sleep
import math

use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())


plane = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
robot = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=True)

#创建一面墙
visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[60, 5, 5])

collision_box_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[60, 5, 5])

wall_id = p.createMultiBody(baseMass=10000, baseCollisionShapeIndex=collision_box_id, baseVisualShapeIndex=visual_shape_id,basePosition=[0, 10, 5])

p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

'''
#直接将机器人丢出去
for _ in range(10):
    p.applyExternalForce(
        objectUniqueId=robot,
        linkIndex=-1,
        forceObj=[-1, 10000, 12000],
        posObj=p.getBasePositionAndOrientation(robot)[0],

        flags=p.WORLD_FRAME
    )
'''

#设置机器人自己动作
for i in range(p.getNumJoints(robot)):
    if "wheel" in p.getJointInfo(robot, i)[1].decode("utf-8"):  # 如果是轮子的关节，则为马达配置参数，否则禁用马达
        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=30,
            force=60
        )
    else:
        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            force=0
        )

useDebugLine = True
hitRayColor = [0, 1, 0]
missRayColor = [1, 0, 0]

raylength = 15#激光长度
rayNum = 16 #激光数量

while True:
    p.stepSimulation()
    #激光检测
    begins, _ = p.getBasePositionAndOrientation(robot)#获取激光发射起点与终点
    rayForms = [begins for _ in range(rayNum)]
    rayTos = [
        [
            begins[0] + raylength * math.cos(2 * math.pi * float(i) / rayNum),
            begins[1] + raylength * math.sin(2 * math.pi * float(i) / rayNum),
            begins[2]
        ]
    for i in range(rayNum)]

    result = p.rayTestBatch(rayForms, rayTos)#调用激光探测函数
    p.removeAllUserDebugItems()

    for index, result in enumerate(result):#激光染色
        if result[0] == -1:
            p.addUserDebugLine(rayForms[index], rayTos[index], missRayColor)
        else:
            p.addUserDebugLine(rayForms[index], rayTos[index], hitRayColor)

    # 碰撞检测
    P_min, P_max = p.getAABB(robot)
    id_tuple = p.getOverlappingObjects(P_min, P_max)
    if len(id_tuple) > 1:
        for ID, _ in id_tuple:
            if ID == robot:
                continue
            else:
                print(f"hit happen! hit object is {p.getBodyInfo(ID)}")

    sleep(1. / 240.)

p.disconnect(serve_id)


