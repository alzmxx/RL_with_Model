from gym_torcs import TorcsEnv
#from sample_agent import Agent
import numpy as np
from ddpg import *
import gc
#gc.enable()

TEST = 100

vision = False
episode_count = 100000
max_steps = 15000
reward = 0
done = False
step = 0

env = TorcsEnv(vision=vision, throttle=True)
agent = DDPG(env)
print("TORCS Experiment Start.")
for episode in range(episode_count):
    print("Episode : ", episode)

    if np.mod(episode, 10) == 0:
        # Sometimes you need to relaunch TORCS because of the memory leak error
        state = env.reset(relaunch=True)
    else:
        state = env.reset()

    for step in range(max_steps):
        action = agent.noise_action(state)
        next_state,reward,done,_ = env.step(action)
        agent.perceive(state,action,reward,next_state,done)
        state = next_state
        if done:
            break
    
    if episode % 1000 == 0 and episode > 100:
        total_reward = 0
        for i in range(TEST):
            if np.mod(i, 10) == 0:
                state = env.reset(relaunch=True)
            else:
                state = env.reset()
            #velocity = [0]*44
            for j in range(max_steps):
                action = agent.action(state)
                next_state,reward,done,_ = env.step(action)
                state = next_state
                total_reward += reward
                if done:
                    break
        ave_reward = total_reward/TEST
        print("episode: ",episode, "Evaluation Average Reward: ", ave_reward)

env.end()  # This is for shutting down TORCS
print("Finish.      lalallalalllllllalllalala")
