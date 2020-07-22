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
import heapq
w_1 = 25.8 / 3600  # value of time ($/hour)
w_2 = 0.868			# oil price ($/L)
d1 = 1000.0			# the distance of d_1 (m)
collision_time_delay = 2
gamma=0.9
baseline_vehicle_fuel_rate = 3.51 * (10 ** (-4)) * (24 ** 3) + 0.407 * 24
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
        if len(self.time_interval_list) > 30:
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
        self.outLanesOBJ=[]
        for lane in incomingLanes:
            laneid=lane.getID()
            self.incLanes.append(laneid)
            self.incLanesOBJ.append(lane)
        for lane in outgoingLanes:
            laneid=lane.getID()
            self.outLanes.append(laneid)
            self.outLanesOBJ.append(lane)
        self.nodePos=nodePosition
        self.lead_vehicle = None
        self.onLaneVehicles=[]
        self.lead_time = 0
        self.temp_vehicle = []
        self.temp_time = 0
        self.totalcost=0
        self.basecost=0
    
    def getSK(self):
        # calculate the predicted headway sk
        temp_time=traci.simulation.getTime()
        time_interval = temp_time - self.lead_time
        return time_interval
        
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
        for lane in self.incLanes:
            if "link" in lane:
                if traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane):
                    return True
        return False

    def restrictDrivingMode(self):
        for i in range(len(self.incLanes)):
            for item in traci.edge.getLastStepVehicleIDs(self.incLanes[i]):
                if distance(self.nodePos,traci.vehicle.getPosition(item))/distance(self.nodePos,self.incLanesOBJ[i].getFromNode().getCoord()) > 1100/(self.incLanesOBJ[i].getLength()):
                    
                    traci.vehicle.setSpeedMode(item,1)
                    traci.vehicle.setSpeed(item, 24.0)


    def coordinate(self,params):
        chase=params%2
        routeSelector=int(params//2)
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
        if not self.temp_vehicle:
            # self.lead_time=traci.simulation.getTime()
            return
        if params>=2:
            print(routeSelector)
            # routeSelector=(int(routeSelector//(1/len(self.outLanes))))%len(self.outLanes)
            if self.ID[8:] in ["5","6","7"]:
                for vehicle in self.temp_vehicle:
                    print(routeSelector,vehicle,self.outLanes[routeSelector])
                    traci.vehicle.setVia(vehicle,[self.outLanes[routeSelector]])
                    traci.vehicle.rerouteEffort(vehicle)
            elif self.ID[8:] =="9":
                for vehicle in self.temp_vehicle:
                    if vehicle[8:12]=="end2":
                        traci.vehicle.setVia(vehicle,[self.outLanes[routeSelector]])
                        traci.vehicle.rerouteEffort(vehicle)
        if self.lead_vehicle==None or self.lead_vehicle not in self.onLaneVehicles:
            self.lead_time=traci.simulation.getTime()
            self.lead_vehicle=self.temp_vehicle[-1]
            return

        # time_interval = self.temp_time - self.lead_time
        if not traci.vehicle.getSpeed(self.lead_vehicle) or not (self.temp_vehicle[0]):
            return 
        if chase==1:
            lead_vehicle_speed = traci.vehicle.getSpeed(self.lead_vehicle)
            lead_vehicle_speed_arrive_time = self.lead_time + d1 / lead_vehicle_speed
            
            follower_vehicle_speed = traci.vehicle.getSpeed(self.temp_vehicle[0])
            follower_vehicle_assumed_arrive_time = self.temp_time + d1 / follower_vehicle_speed

            time_deduction = follower_vehicle_assumed_arrive_time - lead_vehicle_speed_arrive_time - collision_time_delay
            set_follower_speed = d1 / (d1 / follower_vehicle_speed - time_deduction)

            # limit the acceleration speed
            for vehicle in self.temp_vehicle:
                if set_follower_speed <= 40.0:
                    traci.vehicle.setSpeedMode(vehicle, 0)
                    traci.vehicle.setSpeed(vehicle, set_follower_speed)
                    set_follower_speed=d1 / (d1 / set_follower_speed - collision_time_delay)
                    index+=1
                else:
                    break
        
        self.lead_vehicle=self.temp_vehicle[-1]
        self.lead_time=self.temp_time
    def baselineNoCoordinate(self,shortestPaths):
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
        if not self.temp_vehicle:
            # self.lead_time=traci.simulation.getTime()
            return
        
        for vehicle in self.temp_vehicle:
            if vehicle[5]=="1":
                start="junction1"
            else:
                start="junction4"
            if vehicle[11]=="1":
                end="junction2"
            else:
                end="junction3"
            self.totalcost+=shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)

    def baselineCoordinate(self,shortestPaths,junctionids):
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
        if not self.temp_vehicle:
            # self.lead_time=traci.simulation.getTime()
            return
        if self.lead_vehicle==None or self.lead_vehicle not in self.onLaneVehicles:
            self.lead_time=traci.simulation.getTime()
            self.lead_vehicle=self.temp_vehicle[-1]
            for vehicle in self.temp_vehicle:
                if vehicle[5]=="1":
                    start="junction1"
                else:
                    start="junction4"
                if vehicle[11]=="1":
                    end="junction2"
                else:
                    end="junction3"
                self.totalcost+=shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
                self.basecost+=shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
            return
        
        lead_vehicle_speed = traci.vehicle.getSpeed(self.lead_vehicle)
        lead_vehicle_speed_arrive_time = self.lead_time + d1 / lead_vehicle_speed
        
        follower_vehicle_speed = traci.vehicle.getSpeed(self.temp_vehicle[0])
        follower_vehicle_assumed_arrive_time = self.temp_time + d1 / follower_vehicle_speed

        time_deduction = follower_vehicle_assumed_arrive_time - lead_vehicle_speed_arrive_time - collision_time_delay
        set_follower_speed = d1 / (d1 / follower_vehicle_speed - time_deduction)

        if self.lead_vehicle[11]==1:
            leadend="junction2"
        else:
            leadend="junction3"
        for vehicle in self.temp_vehicle:
            if vehicle[5]=="1":
                    start="junction1"
            else:
                start="junction4"
            if vehicle[11]=="1":
                end="junction2"
            else:
                end="junction3"
            self.basecost+=shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
            start=self.ID
            # if vehicle[11]=="1":
            #     end="junction2"
            # else:
            #     end="junction3"
            print(vehicle,"originally spend",(traci.lane.getLength(traci.vehicle.getLaneID(vehicle)))/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000))
            #cost already spent before detector
            
            costBeforeChasing=(traci.lane.getLength(traci.vehicle.getLaneID(vehicle))-1000)/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
            self.totalcost+=costBeforeChasing
            
            if set_follower_speed <= 40.0:
                
                acc=False
                # cost of chasing
                chasingCost=1000/set_follower_speed*(w_1+(3.51 * (10 ** (-4)) * (set_follower_speed ** 3) + 0.407 * set_follower_speed)*w_2/1000)
                # print(costBeforeChasing,chasingCost,set_follower_speed)
                best=(traci.lane.getLength(traci.vehicle.getLaneID(vehicle)))/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+shortestPaths[(start,leadend)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
                bestJunction=start
                splitEdge=traci.vehicle.getLaneID(vehicle)
                if end==leadend:
                    self.totalcost+=chasingCost+shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)*.9
                    print(vehicle,"will follow the previous car down the same track",(traci.lane.getLength(traci.vehicle.getLaneID(vehicle))-1100)/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+chasingCost+shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)*.9)
                    traci.vehicle.setSpeedMode(vehicle, 0)
                    traci.vehicle.setSpeed(vehicle, set_follower_speed)
                    set_follower_speed=d1 / (d1 / set_follower_speed - collision_time_delay)
                    continue
                else:
                    for junction in junctionids:
                        # trying to get best cost out of dynamic programming
                        if (start,junction) in shortestPaths and (junction,end) in shortestPaths and (junction,leadend) in shortestPaths:
                            attempt=costBeforeChasing+chasingCost+shortestPaths[(start,junction)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)*1.9+shortestPaths[(junction,leadend)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+shortestPaths[(junction,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
                            if attempt<best:
                                best=attempt
                                bestJunction=junction
                                splitEdge=shortestPaths[(start,junction)][1]
                                acc=True
                if acc==True:
                    traci.vehicle.setSpeedMode(vehicle, 0)
                    traci.vehicle.setSpeed(vehicle, set_follower_speed)
                    traci.vehicle.setVia(vehicle,[splitEdge])
                    traci.vehicle.rerouteEffort(vehicle)
                    self.totalcost+=chasingCost+shortestPaths[(start,bestJunction)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)*0.9+shortestPaths[(bestJunction,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
                    print(vehicle,"will be in platoon until",bestJunction,"would spend",(traci.lane.getLength(traci.vehicle.getLaneID(vehicle))-1100)/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+chasingCost+shortestPaths[(start,bestJunction)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)*0.9+shortestPaths[(bestJunction,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000),"by going",shortestPaths[(start,bestJunction)][0]+shortestPaths[(bestJunction,end)][0],"splitting at",splitEdge)
                    set_follower_speed=d1 / (d1 / set_follower_speed - collision_time_delay)
                else:
                    self.totalcost+=1100/24*(w_1+(3.51 * (10 ** (-4)) * (24 ** 3) + 0.407 * 24)*w_2/1000)
                    self.totalcost+=shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
                    print(vehicle,"will not in platoon and would spend",(traci.lane.getLength(traci.vehicle.getLaneID(vehicle))-1100)/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+1100/24*(w_1+(3.51 * (10 ** (-4)) * (24 ** 3) + 0.407 * 24)*w_2/1000)+shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000))
                index+=1
                
            else:
                self.totalcost+=1100/24*(w_1+(3.51 * (10 ** (-4)) * (24 ** 3) + 0.407 * 24)*w_2/1000)
                self.totalcost+=shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)
                print(vehicle,"would not in platoon and spend",(traci.lane.getLength(traci.vehicle.getLaneID(vehicle))-1100)/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000)+1100/24*(w_1+(3.51 * (10 ** (-4)) * (24 ** 3) + 0.407 * 24)*w_2/1000)+shortestPaths[(start,end)][0]/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000))
                
                # break
    
        self.lead_vehicle=self.temp_vehicle[-1]
        self.lead_time=self.temp_time


import numpy as np
from gym import spaces
class network:
    
    def __init__(self,path,ui,sumocfgPath,steptime=500):
        net=sumolib.net.readNet(path)
        self.junctions=[]
        self.junctionids=[]
        self.lanes={}
        self.junctionToEdge={}
        self.splitEdge={}
        self.ui=ui
        self.sumocfgPath=sumocfgPath
        for node in net.getNodes():
            if "junction" in node.getID():
                self.junctions.append(junction(node.getID(),node.getIncoming(),node.getOutgoing(),node.getCoord()))
                self.junctionids.append(node.getID())
        self.sk=[0]*len(self.junctions)
        for edge in net.getEdges():
            if "link" in edge.getID():
                self.lanes[edge.getID()]=lane(edge.getID())
                self.junctionToEdge[(edge.getFromNode().getID(),edge.getToNode().getID())]=edge.getID()
        self.action_space=spaces.MultiDiscrete([2,2,2,4,4,2,2,4,2,2,2,4,2])
        self.steptime=steptime
        self.observation_space=spaces.Box(np.array([0]*(len(self.lanes)-1)),np.array([1]*(len(self.lanes)-1)))
        self.dpres={}
        self.shortestDist={}
        self.initGraph()
    def initGraph(self):
        for junction in self.junctions:
            for outlane in junction.outLanesOBJ:
                if "junction" in outlane.getToNode().getID():
                    # if junction.ID in self.dpres:
                    #     self.dpres[junction.ID].append((outlane.getLength()/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000),outlane.getToNode().getID()))
                    # else:
                    #     self.dpres[junction.ID]=[((outlane.getLength()/24*(w_1+baseline_vehicle_fuel_rate*w_2/1000),outlane.getToNode().getID()))]
                    if junction.ID in self.dpres:
                        self.dpres[junction.ID].append((outlane.getLength(),outlane.getToNode().getID()))
                    else:
                        self.dpres[junction.ID]=[(outlane.getLength(),outlane.getToNode().getID())]
        for junction in self.junctions:
            self.dijkstra(junction.ID)
    def dijkstra(self,start):
        res=[]
        heapq.heappush(res,(0,start))
        while res:
            cur=heapq.heappop(res)
            if not (start,cur[1]) in self.shortestDist:
                if cur[1] in self.dpres:
                    for to in self.dpres[cur[1]]:
                        heapq.heappush(res,(to[0]+cur[0],to[1],self.junctionToEdge[(cur[1],to[1])]))
                if cur[1]!=start:
                    self.shortestDist[(start,cur[1])]=[cur[0],cur[2]]
    def step(self,params):
        traci.simulationStep()
        for vehicle in traci.vehicle.getIDList():
            vehicle_item_type = traci.vehicle.getTypeID(vehicle)
            if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                traci.vehicle.setColor(vehicle,(0,255,0))
            else:
                traci.vehicle.setColor(vehicle,(255,0,100))
        return_reward=0
        for i in range(len(self.junctions)):
            self.junctions[i].restrictDrivingMode()
            if self.junctions[i].detectArrival():
    
                # simulation method
                # if curSK > 40.0:
                #     if params%2 == 0:
                #         self.junctions[i].coordinate(0)
                # else:
                #     if params%2 == 0:
                #         self.junctions[i].coordinate(0)
                #     else:
                #         self.junctions[i].coordinate(1)
                # calculatedCost=self.junctions[i].coordinate(params)
                self.junctions[i].coordinate(params[i])
                self.sk[i]=self.junctions[i].getSK()
                # return_reward-=calculatedCost
        observation=self.sk
        if traci.simulation.getTime()<86000:
            done=False
        else:
            done=True
        return observation, -return_reward, done, {}
    
    def baselineStep(self):
        traci.simulationStep()
        for vehicle in traci.vehicle.getIDList():
            vehicle_item_type = traci.vehicle.getTypeID(vehicle)
            if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                traci.vehicle.setColor(vehicle,(0,255,0))
            else:
                traci.vehicle.setColor(vehicle,(255,0,100))
        return_reward=0
        for i in range(len(self.junctions)):
            self.junctions[i].restrictDrivingMode()
            if self.junctions[i].detectArrival() and self.junctions[i].ID[8] in ["5","9"]:
                self.junctions[i].baselineCoordinate(self.shortestDist,self.junctionids)
                # self.junctions[i].baselineNoCoordinate(self.shortestDist)
        
            
    def render(self, mode='human', close=False):
        return

    
    def reset(self):
        traci.close()
        gr.generate_routefile()
        traci.start([sumolib.checkBinary(self.ui), '-c', os.path.join(self.sumocfgPath)])
        simpla.load("data/simpla.cfg.xml")
        for junction in self.junctions:
            for lane in self.lanes:
                self.lanes[lane].reset()
        self.sk=[0]*len(self.junctions)
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
    sumoBinary = checkBinary('sumo-gui')


    # begin the simulation

    #print('Current threshold: ', threshold)

    # choose random seed values
    #seed=random.randint(2,23423)
    seed = 16042
    #gs.random_experiment(seed)

                        
    # generate the vehicle departure time and departure lane
    #platoon_index = gr.generate_routefile()

    # generate the final SUMO file, include net file and vehicle file
    traci.start([sumoBinary, '-c', os.path.join('data', 'C:/Users/Francmeister/Desktop/rl_with_model_2/Nguyen-Dupuis/Nguyen.sumocfg')])

    simpla.load("data/simpla.cfg.xml")
    mgr=simpla._mgr
    newnet=network("C:/Users/Francmeister/Desktop/rl_with_model_2/Nguyen-Dupuis/newND.net.xml","sumo",'C:/Users/Francmeister/Desktop/rl_with_model_2/Nguyen-Dupuis/Nguyen.sumocfg')
    totalcost=0
    totalfuel=0
    totaltime=0
    print(newnet.dpres)
    print(newnet.shortestDist)
    for i in range(len(newnet.junctions)):
        print(i,newnet.junctions[i].ID)
    for k in range(12000):
        # print(newnet.action())
        newnet.baselineStep()
    traci.close()
    cost=0
    basecost=0
    for junction in newnet.junctions:
        print(junction.ID,junction.totalcost)
        if junction.ID[8] in ["5","9"]:
            cost+=junction.totalcost
            basecost+=junction.basecost
    print(cost)
    print(basecost)
        
