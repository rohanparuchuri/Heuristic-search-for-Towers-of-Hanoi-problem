# Heuristic search for Towers of Hanoi problem:

The Towers of Hanoi is a classic problem in computer science and mathematics. This program extends the traditional 3-peg problem to 4 pegs and uses the A* search algorithm to find the optimal solution. The heuristic used is the 'distance to goal position,' which helps in efficiently finding the solution.

## Features
* Solves the Towers of Hanoi problem with 4 pegs.
* Uses A* algorithm for optimal search.
* Flexible to handle any number of disks.
* Provides step-by-step solution for the problem.

## Algorithm
* A* is a popular search algorithm used in pathfinding and graph traversal. It efficiently finds the shortest path to the goal by combining the advantages of Dijkstra's Algorithm and Greedy Best-First-Search.
* In this program, the cost for moving from one state to another is considered to be unit cost.

## Heuristic Explanation
#### Distance Heuristic
*The heuristic used in this program is the 'distance to goal position.' It estimates the cost to reach the goal state from the current state. This heuristic is admissible and consistent, ensuring optimality of the A* algorithm.
  
#### Misplaced Disks Heuristic
*The misplaced disk heuristic is the number of disks that are not in their goal position. It provides an estimate of how far the current state is from the goal state. This heuristic is admissible and consistent, ensuring the optimality of the A* algorithm.

* Distance heuristic visits a lot less states than Misplaced Disks Heuristic, but tend to find more costly solutions.
* I've used distance heuristic, but I've also wrote code for misplaced disks heuristic. Try both heuristics and compare the differences yourself.

### Input
* The programs takes number of disks as the input.

### Output
* The program outputs the number of states visited, cost to reach the goal state and path to goal

### Installation
To use this program, you need to have Python installed on your machine. You can install the required dependencies using pip.
