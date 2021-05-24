import gym
import numpy as np
import time
import sys

num_episodes = 5000  # 共进行5000场游戏
max_number_of_steps = 10  # 每场游戏最大步数

# 以栈的方式记录成绩
goal_average_steps = 100  # 平均分
num_consecutive_iterations = 100  # 栈的容量
last_time_steps = np.zeros(num_consecutive_iterations)  # 只存储最近100场的得分（可以理解为是一个容量为100的栈）

env = gym.make('MyEnvrobt-v0')

# q_table是一个256*2的二维数组
# 离散化后的状态共有4^4=256中可能的取值，每种状态会对应一个行动
# q_table[s][a]就是当状态为s时作出行动a的有利程度评价值
# 我们的AI模型要训练学习的就是这个映射关系表
# 这里的4*4=16是棋盘上棋子的位置数量，第二个参数的4为每个位置对应的4个方向的可能操作。
# q_table的纵坐标是state可能出现的情况之和，横坐标为对应每种state可以做出的action
# 而取值是每种action对于每种state有利程度的评价值
# q_table = np.loadtxt("q_table.txt", delimiter=",")

q_table = np.random.uniform(low=-1, high=1, size=(4 * 4, 4))


# 在这之间随机取值

# 根据本次的行动及其反馈（下一个时间步的状态），返回下一次的最佳行动
# epsilon_coefficient为贪心策略中的ε，取值范围[0,1]，取值越大，行为越随机
# 当epsilon_coefficient取值为0时，将完全按照q_table行动。故可作为训练模型与运用模型的开关值。
def get_action(state, action, observation, reward, episode, epsilon_coefficient=0.0):
    # print(observation)
    next_state = observation
    epsilon = epsilon_coefficient * (0.99 ** episode)  # ε-贪心策略中的ε
    if epsilon <= np.random.uniform(0, 1):
        next_action = np.argmax(q_table[next_state])
    else:
        next_action = np.random.choice([0, 1, 2, 3])
    # -------------------------------------训练学习，更新q_table----------------------------------
    alpha = 0.2  # 学习系数α
    gamma = 0.99  # 报酬衰减系数γ
    q_table[state, action] = (1 - alpha) * q_table[state, action] + alpha * (
            reward + gamma * q_table[next_state, next_action])
    # -------------------------------------------------------------------------------------------
    return next_action, next_state


timer = time.time()  # 返回时间单位是秒
for episode in range(num_episodes):
    env.reset()  # 初始化本场游戏的环境
    episode_reward = 0  # 初始化本场游戏的得分
    q_table_cache = q_table  # 创建q_table还原点，如若训练次数超次，则不作本次训练记录。

    for t in range(max_number_of_steps):
        env.render()  # 更新并渲染游戏画面
        state = env.state
        action = np.argmax(q_table[state])
        observation, reward, done, info = env.step(action)  # 进行活动,#并获取本次行动的反馈结果
        action, state = get_action(state, action, observation, reward, episode, 0.5)  # 作出下一次行动的决策
        episode_reward += reward

        if done:
            np.savetxt("q_table.txt", q_table, delimiter=",")
            print(
                '已完成 %d 次训练，本次训练共进行 %d 步数。episode_reward：%d，平均分： %f' % (episode, t + 1, reward, last_time_steps.mean()))
            last_time_steps = np.hstack((last_time_steps[1:], [reward]))  # 更新最近100场游戏的得分stack
            break
    q_table = q_table_cache  # 超次还原q_table

    print('已完成 %d 次训练，本次训练共进行 %d 步数。episode_reward：%d，平均分： %f' % (episode, t + 1, reward, last_time_steps.mean()))
    last_time_steps = np.hstack((last_time_steps[1:], [reward]))  # 更新最近100场游戏的得分stack；np.hstack():在水平方向上平铺

    if (last_time_steps.mean() >= goal_average_steps):
        np.savetxt("q_table.txt", q_table, delimiter=",")
        print('用时 %d s,训练 %d 次后，模型到达测试标准!' % (time.time() - timer, episode))

        env.close()

        sys.exit()

env.close()
sys.exit()
