import os
import sys
import optparse
import traci

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

if __name__ == "__main__":
    options = get_options()   
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    traci.start([sumoBinary, "-c", "Nguyen.sumocfg"])
    for step in range(0,500):
#        traci.vehicle.setSpeed('a12.5',10)
#        print(traci.vehicle.getIDList())
        #print(traci.inductionloop.getIDList())
        # print(traci.edge.getIDList())
        # print(traci.vehicle.getIDList())
        #print(traci.inductionloop.getVehicleData('abc'))
        #print(traci.inductionloop.getVehicleData('e1Detector_-L3_1_0'))
        print(traci.edge.getLastStepVehicleIDs('link1'))
        print(traci.simulation.getDeltaT())
        # if traci.edge.getLastStepVehicleIDs("link1"):
        #     print(traci.edge.getLastStepVehicleIDs("link1"))
        #     print(traci.vehicle.getTypeID(traci.edge.getLastStepVehicleIDs("link1")[-1]))
        #     print(traci.vehicle.getDistance(traci.edge.getLastStepVehicleIDs("link1")[-1]))
        
        traci.simulationStep()
    traci.close()