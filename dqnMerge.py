import numpy as np
import gym

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory
import pyENV_merge as pyENV
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
traci.start([sumoBinary, '-c', os.path.join("Nguyen-Dupuis/merge.sumocfg")])

simpla.load("data/simpla.cfg.xml")
mgr=simpla._mgr
env=pyENV.network("Nguyen-Dupuis/merge.net.xml","sumo","Nguyen-Dupuis/merge.sumocfg")
# env=pyENV.network("ND/newND.net.xml","sumo","ND/test.sumocfg")

ENV_NAME = 'Merge'


# Get the environment and extract the number of actions.
nb_actions = env.action_space.n

# Next, we build a very simple model.
model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(nb_actions))
model.add(Activation('linear'))
print(model.summary())

# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
# even the metrics!
memory = SequentialMemory(limit=50000, window_length=1)
policy = BoltzmannQPolicy()
dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10,
               target_model_update=1e-2, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

try:
    dqn.load_weights('dqn_{}_weights.h5f'.format(ENV_NAME))
    print("loaded weights file")
except:
    raise Exception("failed to load weights file")

# Okay, now it's time to learn something! We visualize the training here for show, but this
# slows down training quite a lot. You can always safely abort the training prematurely using
# Ctrl + C.
logdir = "logs/scalars/" + datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = keras.callbacks.TensorBoard(log_dir=logdir)
# Okay, now it's time to learn something! We visualize the training here for show, but this
# slows down training quite a lot. You can always safely abort the training prematurely using
# Ctrl + C.

dqn.fit(env, nb_steps=2500, nb_max_episode_steps=250, visualize=True, verbose=2,callbacks=[tensorboard_callback])

# # After training is done, we save the final weights.
dqn.save_weights('dqn_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

# Finally, evaluate our algorithm for 5 episodes.
# dqn.test(env, nb_episodes=1, nb_max_episode_steps=5,callbacks=[tensorboard_callback])
