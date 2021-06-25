#机器人读取
import pybullet as p
import time
import pybullet_data

#连接物理引擎
physicsCilent = p.connect(p.GUI)
print(p.getConnectionInfo(physicsCilent))
#渲染1(开头)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)#去除控件

#添加模型路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#设置重力
p.setGravity(0, 0, -9.8)

#地板
plantId = p.loadURDF("plane.urdf")#useMaximalCoordinates=True能将模型运行更快，但可能造成精度丢失

#加载机器人，并设置加载的机器人的位姿
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

#设置机器人重力方向
p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
#渲染2（结尾）
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

'''
#使用stepSimulation方法，正向动力学进行步进模拟
#1000次交互，每次交互停顿1/240
for i in range(1000):
    p.stepSimulation()
    time.sleep(1 / 240)

#获取位置与方向
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print("-" * 20)
print(f"机器人坐标为：{cubePos}\n 机器人的方向：{cubeOrn}")
print("_" * 20)
'''

##使用setRealTimeSimulation实时模拟
p.setRealTimeSimulation(1)
while True:
    pass

p.disconnect()

