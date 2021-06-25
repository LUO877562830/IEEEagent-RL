#DDPG 框架（一个简单的RL框架，很好的可读性与拓展性）
env = BuildEnv()
actor = PolicyNetwork()
critic = ValueNetwork()
buffer = ExperimenceRelayBuffer()

for i in range(training_loop):
    state = env.reset()

    #expore in env
    for _ in range(max_step):
        next_state, reward, done, info_dict = env.step(action)
        buffer.append((state, reward, done, next_state))
        state = next_state
        if done:
            break

    #update network parameters
    for _ in range(...):
        batch_date = buffer.random_sample()
        Q_table = ..
        critic_object = critic_loss = criterion(Q_label, critic(...))
        actor_object = Q_value_set = critic(state, actor(...))
        optimizer(network_parameters, object, ...).backward()

    # evaluate the policy
    if i % time_step == 0:
        episode_return = evaluate(env, actor)

    if stop_training:
        break

save_model(network_parameters)
