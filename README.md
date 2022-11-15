# Swarm_PSO
This project shows proposed solution to problem specified as:  
"A swarm of robots is tasked with overcoming an obstacle in the shortest possible time. During the passage, the robots have increased probability of receiving damage proportional to the distance between the robots. A solution to the problem should be proposed using the PSO algorithm."  
This problem is the only requirement we've got.  
  
We assume that:
-> The number of robots in the swarm is known  
-> The appearance of the map with obstacles that the robots must overcome is known  
-> The maximum speed of the robot is known, the minimum speed is 0  
-> The size of the robot is equal to one point on the map  
-> The number of robots does not change during the run  
-> The starting area of the swarm and the end point are known  

## Proposed solution
To solve the problem, the A* algorithms were combined to determine the route and the PSO swarm intelligence algorithm to optimize the trip taking into account the distance between the robots and the time to traverse the map. 
The main steps of the program are:  
-> Initialization  
-> Leader selection  
-> Position update  
