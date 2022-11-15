# Swarm_PSO
Matlab project.  
  
This project shows proposed solution to problem specified as:  
"A swarm of robots is tasked with overcoming an obstacle in the shortest possible time. During the passage, the robots have increased probability of receiving damage proportional to the distance between the robots. A solution to the problem should be proposed using the PSO algorithm."  
  
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
  
## Initialization
The user must set the appropriate parameters to run the program, such as:  
-> Map  
-> Maximum speed of the robot (maximum number of fields a robot can cover in one step)  
-> Number of robots in the swarm   
-> Boundaries of the start area and the destination point  
-> Area of special caution, i.e. the area beyond which a penalty will be applied to the robot's position for approaching other individuals (also in number of map fields)  
   
An example of map starting and finish areas:  
![start_end](https://user-images.githubusercontent.com/39136856/201978549-a879e062-7c6f-47c1-ab7b-133268241262.png)
  
An example of the map on which the robots move:  
![example](https://user-images.githubusercontent.com/39136856/201978275-1fb0ad51-f025-4e00-af35-815a96d32539.png)

The blue cross indicates the current position of the robot.  
The red circle indicates the maximum position that robot can reach in one step.  
Green circles are the area of special caution.  

## Leader selection
Since counting for the closest route for each robot would be too heavy computationally, the swarm leader tactic was used.  
The swarm leader is the robot having the shortest route to the selected endpoint at the start. The other robots in the swarm will be following it at a sufficiently close distance, taking into account the distance to other individuals.    
The swarm leader can move freely and have his move calculated first.  
  
![image](https://user-images.githubusercontent.com/39136856/201979667-6d367f25-e6c0-4053-b711-86a3f66e966f.png)
  
   
## Position update 
After the leader moves, the best step is determined for other robots in the swarm. The priority of movement is determined by the distance of the swarm from the leader. The units move in order from the closest to the furthest from leader.  
The most optimal position is selected using the PSO algorithm seeking the optimal solution within the limits of possible robot movement.  
The penalty function of the PSO algorithm takes into account the position of the robot (minimize distance between leader and itself) and the distances to the other individuals (special caution area). 
  
An example of PSO algorithm looking for the best place for robot to move:  
  
![image](https://user-images.githubusercontent.com/39136856/201981636-41141025-1474-4a9e-802d-a822655e21eb.png)
   
Since there may be a change in the robot ranking created based on the distance from the leader during the swarm movement, this ranking is updated every 5 steps. This value can be changed by the user.  
This procedure improves the smoothness of swarm movement.  
  
Example of swarm movement is included in folder: "Wyniki".
