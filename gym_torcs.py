import gym
from gym import spaces
import numpy as np
# from os import path
import snakeoil3_gym as snakeoil3
import numpy as np
import copy
import collections as col
import os
import time
#import math
PI= 3.14159265359

class TorcsEnv:
    terminal_judge_start = 500  # Speed limit is applied after this step
    termination_limit_progress = 5  # [km/h], episode terminates if car is running slower than this limit
    default_speed = 200.0

    initial_reset = True
   
    #sway_current = 0


    def __init__(self, vision=False, throttle=False, gear_change=False):
       #print("Init")
        self.vision = vision
        self.throttle = throttle
        self.gear_change = gear_change

        self.initial_run = True

        ##print("launch torcs")

        os.system('sudo pkill torcs')
        time.sleep(0.5)
        if self.vision is True:
            os.system('torcs -nofuel -nodamage -nolaptime  -vision &')
        else:
            os.system('sudo torcs  -nofuel -nodamage -nolaptime &')
            #os.system('sudo torcs -nofuel -nodamage -nolaptime -r /home/xiongxi/.torcs/config/raceman/quickrace.xml &')
        time.sleep(0.5)
        os.system('sh autostart.sh')
        time.sleep(0.5)


        """
        # Modify here if you use multiple tracks in the environment
        self.client = snakeoil3.Client(p=3101, vision=self.vision)  # Open new UDP in vtorcs
        self.client.MAX_STEPS = np.inf

        client = self.client
        client.get_servers_input()  # Get the initial input from torcs

        obs = client.S.d  # Get the current full-observation from torcs
        """
        if throttle is False:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,))
        else:
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,))

        if vision is False:
            high = np.hstack((np.ones(5)*1.,1,1,1,1,np.ones(19)*1.,np.ones(4)*1.,1,1))
            low = np.hstack((np.ones(5)*0.,0,0,0,0,np.ones(19)*0.,np.ones(4)*0.,-1,-1))
            
            '''
            high = np.array([1., np.inf, np.inf, np.inf, 1., np.inf, 1., np.inf])
            low = np.array([0., -np.inf, -np.inf, -np.inf, 0., -np.inf, 0., -np.inf])
            '''
            self.observation_space = spaces.Box(low=low, high=high)
        else:

            high = np.array([1., np.inf, np.inf, np.inf, 1., np.inf, 1., np.inf, 255])
            low = np.array([0., -np.inf, -np.inf, -np.inf, 0., -np.inf, 0., -np.inf, 0])
            self.observation_space = spaces.Box(low=low, high=high)

    def step(self, u1):
        #print("Step")
        alpha = 0.4
        beta = 0.3
        gama = 0.3
        
        '''
        alpha = 1.0
        beta = 0.0
        gama = 0.0
        '''
        # convert thisAction to the actual torcs actionstr
        client = self.client

        this_action = self.agent_to_torcs(u1)

        # Apply Action
        action_torcs = client.R.d

        # Steering
        
        control_steer = client.S.d['angle']*10 / PI
        control_steer -= client.S.d['trackPos']*2.0
       
        potential_steer = 0.
        potential_acc = 0.
        
        
        for i in range(36):
            potential_steer += (-20.0)/((client.S.d['opponents'][i])**1.5)*np.sin((i+1)*10.0*PI/180.0)
            potential_acc += 10.0/((client.S.d['opponents'][i])**1.5)*np.cos((i+1)*10.0*PI/180.0)
        
        action_torcs['steer'] = alpha*this_action['steer'] + beta*control_steer + gama*potential_steer  

        #  Simple Autnmatic Throttle Control by Snakeoil

        target_speed = self.default_speed

        control_acc = action_torcs['accel']

        if client.S.d['speedX'] < target_speed - abs((client.R.d['steer']*200.0)):
           control_acc += .01
        else:
            control_acc -= .01

        if control_acc > 1.0:
            control_acc = 1.0

        if client.S.d['speedX'] < 10:
            control_acc += 1/(client.S.d['speedX']+.1)

        if (abs((client.S.d['wheelSpinVel'][2]+client.S.d['wheelSpinVel'][3]) - (client.S.d['wheelSpinVel'][0]+client.S.d['wheelSpinVel'][1])) > 5):
            control_acc -= .2

        action_torcs['accel'] = alpha*this_action['accel'] + beta*control_acc + gama*potential_acc
        
        print('Steer: ', this_action['steer'], control_steer, potential_steer)
        print('Acc: ', this_action['accel'], control_acc, potential_acc)        
        #  Automatic Gear Change by Snakeoil
        if self.gear_change is True:
            action_torcs['gear'] = this_action['gear']
        else:
            #  Automatic Gear Change by Snakeoil is possible
            action_torcs['gear'] = 1
            
            if client.S.d['speedX'] > 80:        #90
                action_torcs['gear'] = 2
            if client.S.d['speedX'] > 120:
                action_torcs['gear'] = 3
            if client.S.d['speedX'] > 160:
                action_torcs['gear'] = 4
            if client.S.d['speedX'] > 200:
                action_torcs['gear'] = 5
            if client.S.d['speedX'] > 250:
                action_torcs['gear'] = 6
            

        # Save the privious full-obs from torcs for the reward calculation
        obs_pre = copy.deepcopy(client.S.d)

        # One-Step Dynamics Update #################################
        # Apply the Agent's action into torcs
        client.respond_to_server()
        # Get the response of TORCS
        client.get_servers_input()

        # Get the current full-observation from torcs
        obs = client.S.d

        # Make an obsevation from a raw observation vector from TORCS
        self.observation = self.make_observaton(obs)

        # Reward setting Here #######################################
        # direction-dependent positive reward
        track = np.array(obs['track'])
        spx = np.array(obs['speedX'])
        progress = spx*np.cos(obs['angle'])
        
        reward = progress/150.0
        #reward = progress

        # Termination judgement #########################
        episode_terminate = False
        
        if track.min() < 0:  # Episode is terminated if the car is out of track
            episode_terminate = True
            client.R.d['meta'] = True
        
        if self.terminal_judge_start < self.time_step: 
            if progress < self.termination_limit_progress:
                episode_terminate = True
                client.R.d['meta'] = True

        if np.cos(obs['angle']) < 0:
            episode_terminate = True
            client.R.d['meta'] = True        

        if client.R.d['meta'] is True:
            self.initial_run = False
            client.respond_to_server()

        self.time_step += 1

        return self.get_obs(), reward, client.R.d['meta'], {}

    def reset(self, relaunch=False):
        #print("Reset")

        self.time_step = 0

        if self.initial_reset is not True:
            self.client.R.d['meta'] = True
            self.client.respond_to_server()

            ## TENTATIVE. Restarting TORCS every episode suffers the memory leak bug!
            if relaunch is True:
                self.reset_torcs()
                print("### TORCS is RELAUNCHED ###")

        # Modify here if you use multiple tracks in the environment
        self.client = snakeoil3.Client(p=3101, vision=self.vision)  # Open new UDP in vtorcs
        self.client.MAX_STEPS = np.inf

        client = self.client
        client.get_servers_input()  # Get the initial input from torcs

        obs = client.S.d  # Get the current full-observation from torcs
        self.observation = self.make_observaton(obs)

        self.last_u = None

        self.initial_reset = False
        return self.get_obs()

    def end(self):
        os.system('pkill torcs')

    def get_obs(self):
        return self.observation

    def reset_torcs(self):
       #print("relaunch torcs")
        os.system('sudo pkill torcs')
        time.sleep(0.5)
        if self.vision is True:
            os.system('sudo torcs -nofuel -nodamage -nolaptime -vision &')
        else:
            os.system('sudo torcs -nofuel -nodamage -nolaptime &')
            #os.system('sudo torcs -nofuel -nodamage -nolaptime -r /home/xiongxi/.torcs/config/raceman/quickrace.xml &')
        time.sleep(0.5)
        os.system('sh autostart.sh')
        time.sleep(0.5)

    def agent_to_torcs(self, u):
        torcs_action = {'steer': u[0]}

        if self.throttle is True:  # throttle action is enabled
            torcs_action.update({'accel': u[1]})

        if self.gear_change is True: # gear change action is enabled
            torcs_action.update({'gear': u[2]})

        return torcs_action


    def obs_vision_to_image_rgb(self, obs_image_vec):
        image_vec =  obs_image_vec
        r = image_vec[0:len(image_vec):3]
        g = image_vec[1:len(image_vec):3]
        b = image_vec[2:len(image_vec):3]

        sz = (64, 64)
        r = np.array(r).reshape(sz)
        g = np.array(g).reshape(sz)
        b = np.array(b).reshape(sz)
        return np.array([r, g, b], dtype=np.uint8)

    def make_observaton(self, raw_obs):
        if self.vision is False:
            names = ['focus',
                     'speedX', 'speedY', 'speedZ',
                     #'opponents',
                     'rpm',
                     'track',
                     'wheelSpinVel',
                     'trackPos',
                     'angle']
            Observation = col.namedtuple('Observaion', names)
            return np.hstack(Observation(focus=np.array(raw_obs['focus'], dtype=np.float32)/200.,
                               speedX=np.array(raw_obs['speedX'], dtype=np.float32)/self.default_speed,
                               speedY=np.array(raw_obs['speedY'], dtype=np.float32)/self.default_speed,
                               speedZ=np.array(raw_obs['speedZ'], dtype=np.float32)/self.default_speed,
                               #opponents=np.array(raw_obs['opponents'], dtype=np.float32)/200.,
                               rpm=np.array(raw_obs['rpm'], dtype=np.float32)/10000.,
                               track=np.array(raw_obs['track'], dtype=np.float32)/200.,
                               wheelSpinVel=np.array(raw_obs['wheelSpinVel'], dtype=np.float32)/400.,
                               trackPos=np.array(raw_obs['trackPos'], dtype=np.float32),
                               angle=np.array(raw_obs['angle'], dtype=np.float32)/3.15)
                             )
        else:
            names = ['focus',
                     'speedX', 'speedY', 'speedZ',
                     #'opponents',
                     'rpm',
                     'track',
                     'wheelSpinVel',
                     'img',
                     'trackPos',
                     'angle']
            Observation = col.namedtuple('Observaion', names)

            # Get RGB from observation
            image_rgb = self.obs_vision_to_image_rgb(raw_obs[names[8]])

            return np.hstack(Observation(focus=np.array(raw_obs['focus'], dtype=np.float32)/200.,
                               speedX=np.array(raw_obs['speedX'], dtype=np.float32)/self.default_speed,
                               speedY=np.array(raw_obs['speedY'], dtype=np.float32)/self.default_speed,
                               speedZ=np.array(raw_obs['speedZ'], dtype=np.float32)/self.default_speed,
                               #opponents=np.array(raw_obs['opponents'], dtype=np.float32)/200.,
                               rpm=np.array(r aw_obs['rpm'], dtype=np.float32)/10000,
                               track=np.array(raw_obs['track'], dtype=np.float32)/200.,
                               wheelSpinVel=np.array(raw_obs['wheelSpinVel'], dtype=np.float32)/400.,
                               img=image_rgb,
                               trackPos=np.array(raw_obs['trackPos'], dtype=np.float32),
                               angle=np.array(raw_obs['angle'], dtype=np.float32)/3.15)
                             )
