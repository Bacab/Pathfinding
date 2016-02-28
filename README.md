# Pathfinding
#Old folder
This is my attempts to create a simple and efficient pathfinding algorithm for the robot I'm working on. 
It computes the Tchebychev distance between each cellule of the array representing the map and the goal. 
Then it computes a path choosing at each step the costless adjacent cellule as its next move.
The algorithm computes a trajectory from the starting point of the robot to the goal and then the inverse.
In the end it chooses the trajectory that was computed in the least steps.
The trajectory is displayed with '*' representing the optimal trajectory computed and '**' representing non optimal choice that
may be necessary to use. '*' that are not connected to the starting point by a continuous line of '*' are to be discarded.

#A_star folder
My attempt to implement the A* algoritm in C. Everything should be working as intended.
Include code from https://fr.wikibooks.org/wiki/Structures_de_donn%C3%A9es_en_C/Les_listes_simples (I hate those linked list)
The explanations on which I used for this algorithm can be found here http://web.mit.edu/eranki/www/tutorials/search/
