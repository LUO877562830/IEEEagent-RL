#引入gym
import gym
import pybullet as p
import pybullet_envs
import pybullet_data
from pybullet_envs.bullet import CartPoleBulletEnv
import time
import sys

cid = p.connect(p.DIRECT)
env = gym.make("CartPoleContinuousBulletEnv-v0")
#env = CartPoleBulletEnv(renders=True, discrete_actions=False)
env.render()
env.reset()

def callback(*params):
    print(params[0])
    print("_" * 20)
    print(params[1])
    sys.exit(-1)

for _ in range(10000):
    time.sleep(1. / 60.)
    action = env.action_space.sample()
    observation, reward, done, _ = env.step(action)

p.disconnect(cid)