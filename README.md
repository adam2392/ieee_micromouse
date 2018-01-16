ieee_micromouse
===============
By: Adam Li, Eric, Matt, Sunny, Lucas
### Languages: C, C++, Arduino
## Algorithms: PID, FloodFill

# Background
Code for micromouse ieee competition 2015. Using PID, flood fill algorithm and programming in C. The idea is to have a mouse navigate through the maze and learn the layout and then run through the maze in the fastest possible time.

# Functions
## 1. PID
This is a control algorithm that balances proportional, integrative and derivative errors with respect to a goal, and modifies input into the system to achieve that goal. It can be stable, unstable or oscillatory. Coefficients for each error should be tuned for the specific task.

## 2. Floodfill
This is an algorithm that keeps a working log of the entire maze by knowing apriori the size of each maze square. It will update the algorithm as the robot moves through the maze. On the second traversal of the maze, the floodfill algorithm can be just used to navigate the maze with the shortest path to the solution.

## 3. Utility
These are functionalities we built in to handle driving the mouse and testing it.
