import math
import gym
from gym import spaces, logger, core
from gym.utils import seeding
import numpy as np
import logging
import random

"""
 gym.Env是gym的环境基类,自定义的环境就是根据自己的需要重写其中的方法；core为env环境基类
 必须要重写的方法有: 
  __init__()：构造函数
  reset()：初始化环境
  step()：环境动作,即环境对agent的反馈
  render()：如果要进行可视化则实现
"""

logger = logging.getLogger(__name__)


class MyEnvrobt(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 2
    }

    def __init__(self):

# ------------------初始化状态和方向--------------#
        self.states = list(range(16))  # 状态空间

        self.states.remove(11)  # list中可以使用remove
        self.states.remove(12)
        self.states.remove(15)

        self.x = [150, 250, 350, 450] * 4
        self.y = [450] * 4 + [350] * 4 + [250] * 4 + [150] * 4

        self.terminate_states = dict()  # 终止状态为字典格式
        self.terminate_states[11] = 1
        self.terminate_states[12] = 1
        self.terminate_states[15] = 1

        self.actions = [0, 1, 2, 3]

        self.rewards = dict();  # 回报的数据结构为字典
        self.rewards['7_1'] = -100.0
        self.rewards['8_1'] = -100.0
        self.rewards['10_3'] = -100.0
        self.rewards['13_2'] = -100.0
        self.rewards['14_3'] = 100.0

        self.t = dict();  # 状态转移的数据格式为字典

#------------------数据转化字典------------------#
        self.size = 4

        for i in range(self.size, self.size * self.size):  # 上、下、左、右各方向寻路
            self.t[str(i) + '_0'] = i - 4  # 向上 有12个位置

        for i in range(self.size * (self.size - 1)):
            self.t[str(i) + '_1'] = i + 4  # 向下也有12位置

        for i in range(1, self.size * self.size):
            if i % self.size == 0:  # 最左端数字
                continue
            self.t[str(i) + '_2'] = i - 1

        for i in range(self.size * self.size):
            if (i + 1) % self.size == 0:  # 最右端数字
                continue
            self.t[str(i) + '_3'] = i + 1

        self.gamma = 0.8  # 折扣因子
        self.viewer = None
        self.state = None

    def _seed(self, seed=None):
        self.np_random, seed = random.seeding.np_random(seed)
        return [seed]

    def getTerminal(self):
        return self.terminate_states

    def getGamma(self):
        return self.gamma

    def getStates(self):
        return self.states

    def getAction(self):
        return self.actions

    def getTerminate_states(self):
        return self.terminate_states

    def setAction(self, s):
        self.state = s

    def step(self, action):
        # 系统当前状态
        state = self.state
        # if state in self.terminate_states:
        #     return state, 0, True, {}
        key = "%d_%d" % (state, action)  # 将状态和动作组成字典的键值

        # 状态转移
        if key in self.t:
            next_state = self.t[key]
        else:
            next_state = state
            r = -100.0
            is_terminal = True
            return next_state, r, is_terminal, {}

        self.state = next_state
        is_terminal = False

        if next_state in self.terminate_states:
            is_terminal = True

        if key not in self.rewards:
            r = 0.0
        else:
            r = self.rewards[key]
        return next_state, r, is_terminal, {}

    def reset(self):
        self.state = self.states[int(random.random() * len(self.states))]
        return self.state

    def render(self, mode='human'):
        from gym.envs.classic_control import rendering
        screen_width = 600
        screen_height = 600

        if self.viewer is None:
            self.viewer = rendering.Viewer(screen_width, screen_height)

            # 创建网格世界
            self.line1 = rendering.Line((100, 100), (500, 100))
            self.line2 = rendering.Line((100, 200), (500, 200))
            self.line3 = rendering.Line((100, 300), (500, 300))
            self.line4 = rendering.Line((100, 400), (500, 400))
            self.line5 = rendering.Line((100, 500), (500, 500))
            self.line6 = rendering.Line((100, 100), (100, 500))
            self.line7 = rendering.Line((200, 100), (200, 500))
            self.line8 = rendering.Line((300, 100), (300, 500))
            self.line9 = rendering.Line((400, 100), (400, 500))
            self.line10 = rendering.Line((500, 100), (500, 500))

            # #创建石柱
            # self.shizhu = rendering.make_circle(40)
            # self.circletrans = rendering.Transform(translation=(250,350))
            # self.shizhu.add_attr(self.circletrans)
            # self.shizhu.set_color(0.8,0.6,0.4)

            # 创建第一个火坑
            self.fire1 = rendering.make_circle(40)
            self.circletrans = rendering.Transform(translation=(450, 250))
            self.fire1.add_attr(self.circletrans)
            self.fire1.set_color(1, 0, 0)

            # 创建第二个火坑
            self.fire2 = rendering.make_circle(40)
            self.circletrans = rendering.Transform(translation=(150, 150))
            self.fire2.add_attr(self.circletrans)
            self.fire2.set_color(1, 0, 0)

            # 创建宝石
            self.diamond = rendering.make_circle(40)
            self.circletrans = rendering.Transform(translation=(450, 150))
            self.diamond.add_attr(self.circletrans)
            self.diamond.set_color(0, 0, 1)

            # 创建机器人
            self.robot = rendering.make_circle(30)
            self.robotrans = rendering.Transform()
            self.robot.add_attr(self.robotrans)
            self.robot.set_color(0, 1, 0)

            self.line1.set_color(0, 0, 0)
            self.line2.set_color(0, 0, 0)
            self.line3.set_color(0, 0, 0)
            self.line4.set_color(0, 0, 0)
            self.line5.set_color(0, 0, 0)
            self.line6.set_color(0, 0, 0)
            self.line7.set_color(0, 0, 0)
            self.line8.set_color(0, 0, 0)
            self.line9.set_color(0, 0, 0)
            self.line10.set_color(0, 0, 0)

            self.viewer.add_geom(self.line1)
            self.viewer.add_geom(self.line2)
            self.viewer.add_geom(self.line3)
            self.viewer.add_geom(self.line4)
            self.viewer.add_geom(self.line5)
            self.viewer.add_geom(self.line6)
            self.viewer.add_geom(self.line7)
            self.viewer.add_geom(self.line8)
            self.viewer.add_geom(self.line9)
            self.viewer.add_geom(self.line10)
            # self.viewer.add_geom(self.shizhu)
            self.viewer.add_geom(self.fire1)
            self.viewer.add_geom(self.fire2)
            self.viewer.add_geom(self.diamond)
            self.viewer.add_geom(self.robot)

        if self.state is None:
            return None

        self.robotrans.set_translation(self.x[self.state], self.y[self.state])

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None


