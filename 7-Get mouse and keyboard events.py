#获取键盘事件和鼠标事件
import pybullet as p
import pybullet_data
import math
from time import sleep

use_serve = True
if use_serve:
    reserve_id = p.connect(p.GUI)
else:
    reserve_id = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())


p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)

textColor = [1, 1, 0]

# 先设置一个空内容
debug_text_id = p.addUserDebugText(
    text="",
    textPosition=[0, 0, 2],
    textColorRGB=textColor,
    textSize=2.5
)

maxV = 30
maxF = 30

t = 2  # 左前或右前的轮子的速度差的倍数


while True:
    p.stepSimulation()
    key_dict = p.getKeyboardEvents()

    if len(key_dict):
        if p.B3G_UP_ARROW in key_dict and p.B3G_LEFT_ARROW in key_dict:  # 左前
            p.setJointMotorControlArray(  # 2,3为右 6,7为左
                bodyUniqueId=robot,
                jointIndices=[2, 3],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV],
                forces=[maxF, maxF]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=robot,
                jointIndices=[6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t],
                forces=[maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="up + left",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )

        elif p.B3G_UP_ARROW in key_dict and p.B3G_RIGHT_ARROW in key_dict:  # 右前
            p.setJointMotorControlArray(  # 2,3为右 6,7为左
                bodyUniqueId=robot,
                jointIndices=[6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV],
                forces=[maxF, maxF]
            )
            p.setJointMotorControlArray(
                bodyUniqueId=robot,
                jointIndices=[2, 3],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t],
                forces=[maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="up + right",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )

        elif p.B3G_UP_ARROW in key_dict:  # 向前
            p.setJointMotorControlArray(
                bodyUniqueId=robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV, maxV, maxV, maxV],
                forces=[maxF, maxF, maxF, maxF]
            )
            debug_text_id = p.addUserDebugText(
                text="up",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )

        elif p.B3G_DOWN_ARROW in key_dict:  # 向后
            p.setJointMotorControlArray(
                bodyUniqueId=robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-maxV / t, -maxV / t, -maxV / t, -maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="down",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )


        elif p.B3G_LEFT_ARROW in key_dict:  # 原地左转
            p.setJointMotorControlArray(
                bodyUniqueId=robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[maxV / t, maxV / t, -maxV / t, -maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="left",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )

        elif p.B3G_RIGHT_ARROW in key_dict:  # 原地右转
            p.setJointMotorControlArray(
                bodyUniqueId=robot,
                jointIndices=[2, 3, 6, 7],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=[-maxV / t, -maxV / t, maxV / t, maxV / t],
                forces=[maxF / t, maxF / t, maxF / t, maxF / t]
            )
            debug_text_id = p.addUserDebugText(
                text="right",
                textPosition=[0, 0, 2],
                textColorRGB=textColor,
                textSize=2.5,
                replaceItemUniqueId=debug_text_id
            )

    else:  # 没有按键，则停下
        p.setJointMotorControlArray(
            bodyUniqueId=robot,
            jointIndices=[2, 3, 6, 7],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[0, 0, 0, 0],
            forces=[0, 0, 0, 0]
        )
        debug_text_id = p.addUserDebugText(
            text="",
            textPosition=[0, 0, 2],
            textColorRGB=textColor,
            textSize=2.5,
            replaceItemUniqueId=debug_text_id
        )

    #鼠标事件
    mouse = p.getMouseEvents()
    if len(mouse):
        print(mouse)
        p.stopStateLogging(reserve_id)

p.disconnect(reserve_id)