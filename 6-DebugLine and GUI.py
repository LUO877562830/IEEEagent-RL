#6. Dug模块与GUI
import pybullet as p
import pybullet_data
import time
import math


use_serve = True
if use_serve:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

froms = [[1, 1, 0], [-1, 1, 0], [-1, 1, 3], [1, 1, 3]]
tos = [[-1, 1, 0], [-1, 1, 3], [1, 1, 3], [1, 1, 0]]
for f, t in zip(froms, tos):
    p.addUserDebugLine(
        lineFromXYZ = f,
        lineToXYZ = t,
        lineColorRGB = [0, 1, 0],
        lineWidth = 2
    )
    p.addUserDebugText(
        text="Destination",
        textPosition=[0, 1, 3],
        textColorRGB=[0, 1, 0],
        textSize=1.2,
    )

    p.addUserDebugText(
        text="I'm LYB",
        textPosition=[0, 0, 1.2],
        textColorRGB=[0, 0, 1],
        textSize=1.2
    )

    p.setDebugObjectColor(
        objectUniqueId=robot,
        linkIndex=-1,
        objectDebugColorRGB=[0, 0, 1]
    )

#添加控件
wheel_link_tuples = [(p.getJointInfo(robot, i)[0], p.getJointInfo(robot, i)[1].decode("utf-8"))   # 0:序号 1:名称
    for i in range(p.getNumJoints(robot))
    if "wheel" in p.getJointInfo(robot, i)[1].decode("utf-8")]

wheel_velocity_params_ids = [p.addUserDebugParameter(
    paramName=wheel_link_tuples[i][1] + " V",
    rangeMin=-50,
    rangeMax=50,
    startValue=0
) for i in range(4)]

wheel_force_params_ids = [p.addUserDebugParameter(
    paramName=wheel_link_tuples[i][1] + " F",
    rangeMin=-100,
    rangeMax=100,
    startValue=0
) for i in range(4)]

#控件按钮，用于resert
btn = p.addUserDebugParameter(
    paramName="reset",
    rangeMin=1,
    rangeMax=0,
    startValue=0
)
previous_btn_value = p.readUserDebugParameter(btn)

p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while True:
    p.stepSimulation()
    # 将控件的参数值作为输入控制机器人，先获取各组控件值
    indices = [i for i, _ in wheel_link_tuples]
    velocities = [p.readUserDebugParameter(param_id) for param_id in wheel_velocity_params_ids]
    forces = [p.readUserDebugParameter(param_id) for param_id in wheel_force_params_ids]
    p.setJointMotorControlArray(
        bodyUniqueId=robot,
        jointIndices=indices,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=velocities,
        forces=forces)

    # 如果按钮的累加值发生变化了，说明clicked了
    if p.readUserDebugParameter(btn) != previous_btn_value:
        # 重置速度
        for i in range(p.getNumJoints(robot)):
            p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, 0, 0)
        # 重置位置
        p.resetBasePositionAndOrientation(robot, [0, 0, 0.5], [0, 0, 0, 1])
        previous_btn_value = p.readUserDebugParameter(btn)

    #添加摄像头
    if p.readUserDebugParameter(btn)!=previous_btn_value:
        p.removeAllUserDebugItems()
        previous_btn_value = p.readUserDebugParameter(btn)
    p.getCameraImage(480,320)

p.disconnect(serve_id)

