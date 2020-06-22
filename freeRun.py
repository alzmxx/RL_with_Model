import sumolib
import traci
import simpla
import os
import subprocess
import sys
import shutil
import random
import numpy as np
import time
import theta_calculation as tc
import queue
import generate_routefile as gr

w_1 = 25.8 / 3600  # value of time ($/hour)
w_2 = 0.868			# oil price ($/L)
d1 = 1000.0			# the distance of d_1 (m)
collision_time_delay = 2
gamma=0.9
	
def distance(coord1,coord2):
    return (abs(coord1[0]-coord2[0])**2+abs(coord1[1]-coord2[1])**2)**0.5
class lane:
    def __init__(self,ID):
        self.ID=ID
        self.time_interval_list=[]
        self.lead_time=0
        self.flow=0
    
    def updateFlow(self):
        # calculate the predicted headway sk
        temp_time=traci.simulation.getTime()
        time_interval = temp_time - self.lead_time
        # calculate the estimated headway
        if len(self.time_interval_list) > 50:
            self.time_interval_list.pop(0)
        self.time_interval_list.append(time_interval)

        estimated_arrval_rate = 0
        gamma_index = 0
        for i_pop_list in range(len(self.time_interval_list)):
            estimated_arrval_rate += self.time_interval_list[len(self.time_interval_list) - 1 - i_pop_list] * (gamma ** gamma_index)
            gamma_index += 1

        self.lead_time=temp_time
        self.flow=1.0 / (estimated_arrval_rate * (1.0 - gamma))

    def reset(self):
        self.time_interval_list.clear()
        self.lead_time=0
        self.flow=0

class junction:
    def __init__(self, ID, incomingLanes,outgoingLanes, nodePosition):
        self.ID = ID
        self.incLanes = []
        self.incLanesOBJ=[]
        self.outLanes = []
        self.inflows={}
        self.inflow_rates={}
        self.outflows={}
        self.outflow_rates={}
        for lane in incomingLanes:
            laneid=lane.getID()
            self.incLanes.append(laneid)
            self.incLanesOBJ.append(lane)
            self.inflows[laneid]=queue.Queue(50)
            self.inflow_rates[laneid]=0
        for lane in outgoingLanes:
            laneid=lane.getID()
            self.outLanes.append(laneid)
            self.outflows[laneid]=queue.Queue(50)
            self.outflow_rates[laneid]=0
        self.nodePos=nodePosition
        self.lead_vehicle = None
        self.onLaneVehicles=[]
        self.lead_time = 0
        self.temp_vehicle = []
        self.temp_time = 0
        self.ini_sk = 0
        self.sk = 0
        
        # self.coordinate_matrix=[None for i in range]
    def getTotalCost(self):
        all_vehicle_list = []
        for lane in self.incLanes:
            all_vehicle_list.append(traci.edge.getLastStepVehicleIDs(lane))
        time_delta = traci.simulation.getDeltaT()
        total_time=0
        total_fuel=0
        for item in all_vehicle_list:
            for vehicle_item in item:
                # vehicle_fuel_rate = traci.vehicle.getFuelConsumption(vehicle_item)
                vehicle_speed=traci.vehicle.getSpeed(vehicle_item)
                vehicle_fuel_rate = 3.51 * (10 ** (-4)) * (vehicle_speed ** 3) + 0.407 * vehicle_speed
                vehicle_item_type = traci.vehicle.getTypeID(vehicle_item)
                if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                    total_fuel += 0.9 * vehicle_fuel_rate * time_delta
                else:
                    total_fuel += vehicle_fuel_rate * time_delta
                total_time += time_delta
        total_cost = total_fuel / 1000.0 * w_2 + total_time * w_1
        return total_cost, total_fuel/1000, total_time


    def detectArrival(self):
        out=[]
        for lane in self.incLanes:
            if "link" in lane:
                if traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane):
                    out.append(lane)
        return out

    def restrictDrivingMode(self):
        for i in range(len(self.incLanes)):
            for item in traci.edge.getLastStepVehicleIDs(self.incLanes[i]):
                if distance(self.nodePos,traci.vehicle.getPosition(item))/distance(self.nodePos,self.incLanesOBJ[i].getFromNode().getCoord()) > 1100/(self.incLanesOBJ[i].getLength()):
                    traci.vehicle.setSpeedMode(item,1)
                    traci.vehicle.setSpeed(item, 24.0)


    def decelerate(self,threshold,C):
        self.ini_sk = C
        # not catch up and decelerate the speed
        time_deduction = C

        follower_vehicle_speed = traci.vehicle.getSpeed(self.temp_vehicle[0])
        set_follower_speed = d1 / (d1 / follower_vehicle_speed - time_deduction)
        for vehicle in self.temp_vehicle:
            if set_follower_speed <= 40.0:
                traci.vehicle.setSpeedMode(vehicle, 0)
                traci.vehicle.setSpeed(vehicle, set_follower_speed)
                # print("decelerate",vehicle,"speed to",set_follower_speed)
                set_follower_speed=d1 / (d1 / set_follower_speed - collision_time_delay)


    def coordinate(self,threshold=30,C=-20,link=None):
        self.temp_vehicle.clear()
        self.onLaneVehicles.clear()
        index=0
        for lane in self.incLanes:
            if (not "end" in lane) and (not "start" in lane):
                for vehicle in traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane):
                    self.temp_vehicle.append(vehicle)
                for vehicle in traci.edge.getLastStepVehicleIDs(lane):
                    self.onLaneVehicles.append(vehicle)
        self.temp_time = traci.simulation.getTime()
        if self.lead_vehicle==None or self.lead_vehicle not in self.onLaneVehicles:
            # print(self.temp_vehicle,"case0")
            self.lead_time=traci.simulation.getTime()
            self.decelerate(threshold,C)
            self.lead_vehicle=self.temp_vehicle[-1]
            return

       
        # calculate the predicted headway sk
        
        time_interval = self.temp_time - self.lead_time
        self.sk = self.ini_sk + time_interval
        # if self.ID=="junction5":
        #     print("leadvehicle:",self.lead_vehicle,"tempvehicles:",self.temp_vehicle,self.sk,self.ini_sk,self.temp_time,self.lead_time)
        if not traci.vehicle.getSpeed(self.lead_vehicle) or not (self.temp_vehicle[0]):
            return 
        if self.sk < threshold:
            
            lead_vehicle_speed = traci.vehicle.getSpeed(self.lead_vehicle)
            lead_vehicle_speed_arrive_time = self.lead_time + d1 / lead_vehicle_speed
            
            follower_vehicle_speed = traci.vehicle.getSpeed(self.temp_vehicle[0])
            follower_vehicle_assumed_arrive_time = self.temp_time + d1 / follower_vehicle_speed

            time_deduction = follower_vehicle_assumed_arrive_time - lead_vehicle_speed_arrive_time - collision_time_delay
            set_follower_speed = d1 / (d1 / follower_vehicle_speed - time_deduction)

            # limit the acceleration speed
            for vehicle in self.temp_vehicle:
                if set_follower_speed <= 40.0:
                    self.ini_sk = self.sk
                
                    traci.vehicle.setSpeedMode(vehicle, 0)
                    traci.vehicle.setSpeed(vehicle, set_follower_speed)
                    # print("accelerate",vehicle,"speed to",set_follower_speed,"sk:",self.sk)
                    set_follower_speed=d1 / (d1 / set_follower_speed - collision_time_delay)
                    index+=1
                    # print(self.temp_time,vehicle,"case1")
                else:
                    break
            
            self.ini_sk = C
            # not catch up and decelerate the speed
            time_deduction = C

            follower_vehicle_speed = traci.vehicle.getSpeed(self.temp_vehicle[0])
            set_follower_speed = d1 / (d1 / follower_vehicle_speed - time_deduction)
            for i in range(index,len(self.temp_vehicle)):
                self.temp_vehicle[i]
                if set_follower_speed <= 40.0:
                    traci.vehicle.setSpeedMode(self.temp_vehicle[i], 0)
                    traci.vehicle.setSpeed(self.temp_vehicle[i], set_follower_speed)
                    # print("decelerate",self.temp_vehicle[i],"speed to",set_follower_speed)
                    set_follower_speed=d1 / (d1 / set_follower_speed - collision_time_delay)
                    # print(self.temp_time,self.temp_vehicle[i],"case2")
                
        else:
            # print(self.temp_time,"case3")
            self.decelerate(threshold,C)
        self.lead_vehicle=self.temp_vehicle[-1]
        self.lead_time=self.temp_time
        if link and self.lead_vehicle:
            traci.vehicle.setVia(self.lead_vehicle,[link])


import numpy as np
from gym import spaces
class network:
    
    def __init__(self,path,ui,sumocfgPath,steptime=300):
        net=sumolib.net.readNet(path)
        self.junctions=[]
        self.lanes={}
        self.ui=ui
        self.sumocfgPath=sumocfgPath
        for node in net.getNodes():
            if "junction" in node.getID():
                self.junctions.append(junction(node.getID(),node.getIncoming(),node.getOutgoing(),node.getCoord()))
        for edge in net.getEdges():
            if "link" in edge.getID():
                self.lanes[edge.getID()]=lane(edge.getID())
        lowVals=np.array([0,-1]*len(self.junctions))
        highVals=np.array([1,0]*len(self.junctions))
        # lowVals=np.array([0,-1])
        # highVals=np.array([1,0])
        # self.action_space = spaces.Box(low=lowVals, high=highVals)
        self.action_space=spaces.Box(low=lowVals, high=highVals)
        self.steptime=steptime
        self.observation_space=spaces.Box(np.array([0]*len(self.lanes)),np.array([1]*len(self.lanes)))
        # self.baseline=self.getBaseline()
        self.reset()

    def step(self,params):
        totalcost=0
        # print(params)
        for i in range(self.steptime):
            
            self.action(params)
            totalcost+=self.getTotalCost()[0]

        observation=self.get_observation()
        # reward=(self.baseline-totalcost)/totalcost
        if traci.simulation.getTime()>8700:
            done=True
        else:
            done=False
        print("params:")
        print(params)
        print("observations:")
        print(observation)
        return observation, (350-totalcost)/50.0, done, {}
    
    def render(self, mode='human', close=False):
        return

    def get_observation(self):
        flows=[]
        for lane in self.lanes:
            flows.append(self.getFlow(lane)*10)
        observation=np.array(flows)
        return observation

    def reset(self):
        traci.close()
        gr.generate_routefile()
        traci.start([sumolib.checkBinary(self.ui), '-c', os.path.join('data',self.sumocfgPath)])
        simpla.load("data/simpla.cfg.xml")
        for junction in self.junctions:
            for lane in self.lanes:
                self.lanes[lane].reset()
        for i in range(1200):
            traci.simulationStep()
            for i in range(len(self.junctions)): 
                self.junctions[i].restrictDrivingMode()
                toUpdate=self.junctions[i].detectArrival()
                if toUpdate:
                    for lane in toUpdate:
                        self.lanes[lane].updateFlow()
    
        return self.get_observation()

    def close(self):
        traci.close()


    def getTotalCost(self):
        total_cost=0
        total_fuel=0
        total_time=0
        for junction in self.junctions:
            data=junction.getTotalCost()
            total_cost+=data[0]
            total_fuel+=data[1]
            total_time+=data[2]
        return total_cost,total_fuel,total_time
    
    # def getBaseline(self):
    #     totalcost=0
    #     for i in range(600):
    #         traci.simulationStep()
    #     for i in range(3600):
    #         traci.simulationStep()
    #         totalcost+=self.getTotalCost()[0]
    #     print("baseline for learning:",totalcost/12)
    #     return totalcost/12
    
    def action(self,params):
        traci.simulationStep()
        for vehicle in traci.vehicle.getIDList():
            vehicle_item_type = traci.vehicle.getTypeID(vehicle)
            if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                traci.vehicle.setColor(vehicle,(0,255,0))
            else:
                traci.vehicle.setColor(vehicle,(255,0,100))
        # threshold=params[0]*50
        # C=params[1]*50
        for i in range(len(self.junctions)): 
            threshold=params[2*i]*80
            C=params[2*i+1]*80*(-1)
            
            self.junctions[i].restrictDrivingMode()
            toUpdate=self.junctions[i].detectArrival()
            if toUpdate:
                self.junctions[i].coordinate(threshold,C)
                for lane in toUpdate:
                    self.lanes[lane].updateFlow()
    def noParamRun(self):
        traci.simulationStep()
        for vehicle in traci.vehicle.getIDList():
            vehicle_item_type = traci.vehicle.getTypeID(vehicle)
            if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                traci.vehicle.setColor(vehicle,(0,255,0))
            else:
                traci.vehicle.setColor(vehicle,(255,0,100))
        for i in range(len(self.junctions)): 
            self.junctions[i].restrictDrivingMode()
            toUpdate=self.junctions[i].detectArrival()
            if toUpdate:
                for lane in toUpdate:
                    self.lanes[lane].updateFlow()
    def getFlow(self,lane):
        return self.lanes[lane].flow



    

# def make(ui,sumocfgPath,netFilePath):
#     try:
#         sys.path.append(os.path.join(os.path.dirname(
#         __file__), '..', '..', '..', '..', "tools"))
#         sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
#         os.path.dirname(__file__), "..", "..", "..")), "tools")) 
        
#     except ImportError:
#         sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")


#     global sumoBinary,netconvertBinary
#     # choose whether to use GUI or not
#     netconvertBinary = checkBinary('netconvert')
#     sumoBinary = checkBinary(ui)


#     # begin the simulation

#     #print('Current threshold: ', threshold)

#     # choose random seed values
#     #seed=random.randint(2,23423)
#     seed = 16042
#     #gs.random_experiment(seed)

                        
#     # generate the vehicle departure time and departure lane
#     #platoon_index = gr.generate_routefile()

#     # generate the final SUMO file, include net file and vehicle file
#     traci.start([sumoBinary, '-c', os.path.join('data', sumocfgPath)])

#     simpla.load("data/simpla.cfg.xml")
#     mgr=simpla._mgr
#     newnet=network(netFilePath,ui,sumocfgPath)

    
#     return newnet
# env = make("sumo-gui",'C:/Users/Francmeister/Desktop/RL_With_SUMO/Nguyen-Dupuis/singleJunction.sumocfg',"C:/Users/Francmeister/Desktop/RL_With_SUMO/Nguyen-Dupuis/singleJunction.net.xml")
if __name__ == "__main__":

	# find SUMO path and start the sumo program
    try:
        sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))
        sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools")) 
        from sumolib import checkBinary
    except ImportError:
        sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")



    # choose whether to use GUI or not
    netconvertBinary = checkBinary('netconvert')
    sumoBinary = checkBinary('sumo')


    # begin the simulation

    #print('Current threshold: ', threshold)

    # choose random seed values
    #seed=random.randint(2,23423)
    seed = 16042
    #gs.random_experiment(seed)

                        
    # generate the vehicle departure time and departure lane
    #platoon_index = gr.generate_routefile()

    # generate the final SUMO file, include net file and vehicle file
    traci.start([sumoBinary, '-c', os.path.join('data', 'C:/Users/Francmeister/Desktop/RL_With_SUMO/Nguyen-Dupuis/Nguyen.sumocfg')])

    simpla.load("data/simpla.cfg.xml")
    mgr=simpla._mgr
    newnet=network("C:/Users/Francmeister/Desktop/RL_With_SUMO/Nguyen-Dupuis/newND.net.xml","sumo-gui",'C:/Users/Francmeister/Desktop/RL_With_SUMO/Nguyen-Dupuis/Nguyen.sumocfg')
    totalcost=0
    totalfuel=0
    totaltime=0
    print(totalcost,totalfuel,totaltime)
    for k in range(6000):
        # print(newnet.action())
        # newnet.action([50,-40]*len(newnet.junctions))
        newnet.noParamRun()
        data=newnet.getTotalCost()
        totalcost+=data[0]
        totalfuel+=data[1]
        totaltime+=data[2]
    traci.close()
    print(totalcost,totalfuel,totaltime)
