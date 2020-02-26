from gym_torcs import TorcsEnv
#from sample_agent import Agent
import numpy as np
from ddpg import *
import gc
gc.enable()

TEST = 100
vision = False
max_steps = 100000
reward = 0
done = False
step = 0
#velocity = [0]*44

# Generate a Torcs environment
env = TorcsEnv(vision=vision, throttle=True)
agent = DDPG(env)

print("TORCS Experiment Start.")

for i in range(TEST):
    state = env.reset()
    total_reward = 0
    #velocity = [0]*44
    for j in range(max_steps):
        action = agent.action(state)
        #print(action)
        next_state,reward,done,_ = env.step(action)
        '''
        next_state,reward,done,_ = env.step(action, velocity)
        #print(j)
        for k in range(8,43):
            if ((next_state[k]*200) < 200) and ((state[k]*200) < 200):
                velocity[k] = (next_state[k]*200) - (state[k]*200)
        '''
        state = next_state
        total_reward += reward        
        if done:
            break
    #print(total_reward)
env.end()  # This is for shutting down TORCS
print("Finish.      lalallalalllllllalllalala")
