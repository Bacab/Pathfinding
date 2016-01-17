# Pathfinding
This is my attempts to create a simple and efficient pathfinding algorithm for the robot I'm working on. 
It computes the Tchebychev distance between each cellule of the array representing the map and the goal. 
Then it computes a path choosing at each step the costless adjacent cellule as its next move.
The algorithm computes a trajectory from the starting point of the robot to the goal and then the inverse.
In the end it chooses the trajectory that was computed in the least steps.
The trajectory is displayed with '*' representing the optimal trajectory computed and '**' representing non optimal choice that
may be necessary to use. '*' that are not connected to the starting point by a continuous line of '*' are to be discarded.
