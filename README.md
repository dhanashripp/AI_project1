# AI_project1

This project contain implementation of DFS, BFS, UCS, A-star search algorithms.
Additional implementation include:

Solution to Corners Problem: 
In order to keep track of visited corners we have maintained a list of booleans, which is updated as and when a corner is visited by the Pacman. While generating successors, we first check if the action to be taken is legal if the wall is not hit by taking that action. If a corner is visited, its status is changed to True. Then, the updated corners list is passed on to successors. For Goal state, we check if all the corners are visited by Pacman
 
Corners Heuristic
This function should always return a number that is a lower bound on the shortest path from the state to a goal of the problem (All corners visited). In order to make sure our heuristic function never overestimate we first visit corner with maximum Manhattan Distance from current position. Due to this property of heuristic function, the heuristic value returned go on decreasing as we reach closer to the goal . Thus both admissibility and consistency of heuristic function are achieved
