import gym
import pybullet as p
import pybullet_envs
import pybullet_data
from gym import spaces
import numpy as np

class CricleDrive(gym.Env):
    def __init__(self, render: bool = False):
        self._render = render

        self.action_space = spaces.Box(
            low=np.array([-10, -10]),
            high=np.array([10, 10]),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=np.array([0, 0]),
            high=np.array([100, np.pi])
        )

        self.reserve_id = p.connect(p.GUI if self._render else p.DIRECT)

        self.target_radius = 3#圆周运动半径

        self.step_num = 0#计数器

    def apply_action(self, action):
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in")

        left_v, right_v = action
        left_v = np.clip(left_v, -10, 10)
        right_v = np.clip(right_v, -10, 10)
        p.setJointMotorControlArray(
            bodyUniqueId = self.robot,
            jointIndices = [3, 2],
            controlMode = p.VELOCITY_CONTROL,
            targetVelocities = [left_v, right_v],
            forces=[10, 10],
            physicsClientId = self.reserve_id
        )

    def get_observation(self):
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in")
        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.robot)

        distance = np.linalg.norm(np.array(basePos))#获取距离
        matrix = p.getMatrixFromQuaternion(baseOri, physicsClientId=self.robot)#获取夹角
        direction_vector = np.array([matrix[0], matrix[3], matrix[6]])
        position_vector = np.array(basePos)
        d_L2 = np.linalg.norm(direction_vector)
        p_L2 = np.linalg.norm(position_vector)

        if d_L2 ==0 or p_L2 == 0:
            return np.array([distance, np.pi])
        angle = np.arccos(np.dot(direction_vector, position_vector) / (d_L2 * p_L2))
        return np.array([distance, angle])

    def reset(self):
        p.resetSimulation(physicsClientId=self.reserve_id)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.2], physicsClientId=self.reserve_id)
        self.plane = p.loadURDF("plane.urdf", physicsClientId=self.reserve_id)
        return self.get_observation()

    def step(self, action):
        self.apply_action(action)
        p.stepSimulation(physicsClientId=self.reserve_id)
        self.step_num += 1
        state = self.get_observation()
        reward = -1 *np.abs(state[0] - self.target_radius) -0.1*np.abs(state[1] - np.pi / 2)
        if state[0] > 5 or self.step_num > 36000:
            done = True
        else:
            done = False
        info = {}
        return state, reward, done, info

    def seed(self, seed=None):
        pass

    def render(self, model='human'):
        pass

    def close(self):
        if self.reserve_id >= 0:
            p.disconnect()
        self.reserve_id = -1


if __name__ == "__main__":
    env = CricleDrive(render=True)
    obs = env.reset()
    p.setRealTimeSimulation(1)


    while True:
        action = np.random.uniform(-10, 10, size=(2, ))
        obs, reward, done, _ = env.step(action)
        if done:
            break

        print(f"state : {obs}, reward : {reward}")
