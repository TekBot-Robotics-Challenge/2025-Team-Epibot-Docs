# A* Algorithm

## What is the A* Algorithm?

The A* algorithm is a popular pathfinding and graph traversal algorithm. It is widely used in robotics and games to find the shortest path between two points, especially when there are obstacles.

## How Does It Work?

A* uses a best-first search and finds the least-cost path from a start node to a goal node. It uses a cost function:

- **f(n) = g(n) + h(n)**
  - **g(n):** The cost to reach node `n` from the start.
  - **h(n):** The estimated cost from node `n` to the goal (heuristic).

The algorithm explores paths that appear to be the most promising, based on the sum of the actual cost and the estimated cost.

## Steps of the Algorithm

1. Add the start node to an open list.
2. Repeat:
   - Pick the node with the lowest `f` value from the open list.
   - If this node is the goal, reconstruct the path and finish.
   - Move the node to the closed list.
   - For each neighbor:
     - If it is not walkable or is in the closed list, skip it.
     - If it is not in the open list, add it and compute its `f`, `g`, and `h` values.
     - If it is already in the open list, check if this path to that node is better (lower `g` value). If so, update its parent and costs.

## Why Use A*?

- Finds the shortest path efficiently.
- Can avoid obstacles.
- Works well in grid-based maps like the robot's arena.

## Example

Suppose the robot needs to go from point A to point B in an arena with walls. The A* algorithm will calculate the best route, avoiding walls and minimizing the distance.

## In the CollectorRobot Class

The CollectorRobot uses the A* algorithm to plan its movement in the arena, ensuring it can reach waste items or the sorter robot efficiently while avoiding obstacles.

---
For more details, see this [GeeksforGeeks article on A*](https://www.geeksforgeeks.org/a-search-algorithm/).