import gym
from gym import spaces

class MySim(gym.Env):
    def __init__(self):
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Discrete(2)

    def step(self, action):
        state = 1

        if action == 2:
            reward = 1
        else:
            reward = -1
        done = True
        info = {}
        return state, reward, done, info

    def reset(self):
        state = 0
        return state

    def render(self, model = 'human'):
        pass

    def seed(self, seed=None):
        pass

if __name__ == "__main__":
    from stable_baselines import PPO2
    from stable_baselines import deepq

    env = MySim()

    model = deepq.DQN(policy="MlpPolicy", env=env)
    model.learn(total_timesteps=10000)

    obs = env.reset()
    # 验证十次
    for _ in range(10):
        action, state = model.predict(observation=obs)
        print(action)
        obs, reward, done, info = env.step(action)
        env.render()