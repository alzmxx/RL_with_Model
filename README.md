# Combining RL with Model-based Control

This is the code with the paper: 
 'Combining Deep Reinforcement Learning and Safety Based Control for Autonomous Driving.'
https://arxiv.org/pdf/1612.00147.pdf'.

#The basic idea here is to combine model-free learning with model-based control to improve long-term control performance.

#The Deep Deterministic Policy Gradient (DDPG) is the model-free learning framework.

#The simulation platform is TORCS.

* The combination algorithm is in gym_torcs.py.


#We implemented the algorithm described in the paper "A Distributed Framework for Coordinated
Heavy-Duty Vehicle Platooning".

#The platform to test the effectiveness of the algorithm was SUMO(simulation of urban mobility) and Traci(the interface of SUMO written in Python).

#We created Nguyen-Dupuis and Sioux Falls networks based on the information provided in the paper "Transportation Research Part B" with the tool NetEdit that comes with sumo and marked all the nodes,edges and links as in the paper.

#Our thesis is based on the assumption that heavyduty vehicles running in platoon would have lower wind resistance, thus they need fewer fuel to operate. Platooning may also reduce the chance of having traffic jams.

#The essense of this algorithm is to find if cars should be coordinated to form platoon and where they should split and go on with their own merry way based on their origin and destination.
#First we use traditional path finding algorithm, Dijkstra Algorithm to find all pairs shortest paths in the network. Then, as the cars that are about to merge into one lane enter the coordination zones, try every other possible split point and calculate the estimated costs for choosing them. If the result is better than just letting them go along uncoordinated, coordinate. Take notes of those formed platoons, the cars within them would not be able to accelerate to chase other cars until they are splitted at the designated split point. 
