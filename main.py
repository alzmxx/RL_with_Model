import numpy as np
from ddpg import *
import gc
#gc.enable()

TEST = 3

vision = False
episode_count = 1000
max_steps = 100
reward = 0
done = False
step = 0

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess
import pyENV_j5 as pyENV
from datetime import datetime
import keras
import sumolib
import traci
import simpla
import os
import sys
from sumolib import checkBinary

try:
    sys.path.append(os.path.join(os.path.dirname(
    __file__), '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools")) 
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")



# choose whether to use GUI or not
netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo')


# begin the simulation

# generate the final SUMO file, include net file and vehicle file
traci.start([sumoBinary, '-c', os.path.join('Nguyen-Dupuis/Nguyen.sumocfg')])

simpla.load("data/simpla.cfg.xml")
mgr=simpla._mgr
env=pyENV.network("Nguyen-Dupuis/newND.net.xml","sumo","Nguyen-Dupuis/Nguyen.sumocfg")

agent = DDPG(env)
print("SUMO Experiment Start.")
for episode in range(episode_count):
    print("Episode : ", episode)

    # if np.mod(episode, 10) == 0:
    #     # Sometimes you need to relaunch TORCS because of the memory leak error
    #     state = env.reset(relaunch=True)
    # else:
    #     state = env.reset()
    state=env.reset()
    
    for step in range(max_steps):
        # print('step: ', step)
        print('state: ', state)
        action = agent.noise_action(state)
        # print("actions:",action)
        next_state,reward,done,_ = env.step(action)
        print("reward:",reward)
        agent.perceive(state,action,reward,next_state,done)
        state = next_state
        if done:
            break
    

    if episode % 10 == 0 and episode > 0:
        total_reward = 0
        for i in range(TEST):
            # if np.mod(i, 10) == 0:
            #     state = env.reset(relaunch=True)
            # else:
            #     state = env.reset()
            env.reset()
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

#env.end()  # This is for shutting down TORCS
print("Finish.      lalallalalllllllalllalala")
