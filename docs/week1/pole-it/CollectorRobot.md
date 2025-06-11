# CollectorRobot Class

## Overview

The `CollectorRobot` class inherits from `Robot` and models an autonomous robot designed to collect waste in an arena. It can move, detect and collect waste, map its environment by sorting waste positions by distance, and plan optimal paths using the A* algorithm.

## Main Attributes

```c++
// Indicates if the robot is currently moving.
bool _is_moving;
// Maximum number of waste items the robot can carry.
int _max_capacity;
// The position of the sorter robot in the arena.
Position _SorterRobotPosition;
// The arena map, where each character represents an element (wall, waste, etc.).
std::vector<std::string> _arenaMap;
//Internal container for storing collected waste.
std::vector<Waste> _container;
```
---

## Public Methods
```c++
CollectorRobot(const std::string& name, int max_capacity, Position SorterRobotPosition, std::vectorstd::string arenaMap);
```

Constructor. Initializes the collector robot with its name, capacity, the sorter robot's position, and the arena map.

```c++
void move(int dx, int dy, int dz) override;
```

Moves the robot by the specified deltas.

```c++
bool isWaste(char character);
```

Returns true if the given character represents a waste item.

```c++
void collect(char character, Position charPos);
```

Collects a waste item at the given position.

```c++
void decharge(std::vector<Waste> &sorterContainer);
```

Empties the robot's container into the sorter robot's container.

```c++
void sortWastesByDistance(Position current, std::vector<Position>& wastes);
```

Environment Mapping: Sorts the list of waste positions by their distance from the current position.

`Note:` This method is used to prioritize which waste items to collect first.

```c++
std::vector<Position> a_star(int y, int x);
```

Computes the optimal path to the target position using the [`A* algorithm`](./AStarAlgorithm.md).

```c++
std::vector<Waste> getContainer() const;
```

Returns the robot's current waste container.

```c++
int getMaxCapacity() const;
```

Returns the robot's maximum carrying capacity.

```c++
Position getSorterRobotPosition() const;
```

Returns the position of the sorter robot.

```c++
std::vector<Position> getWastePositions();
```

Returns a list of all detected waste positions in the arena.

`Note:` This method is used to map the environment

```c++
~CollectorRobot();
```

Destructor.

## Private Methods (A* Utilities)

```c++
std::vector<std::vector<t_astar_node*>> create_astar_grid(int height, int width, const Position& goal);

void delete_astar_grid(std::vector<std::vector<t_astar_node*>>& grid);

t_astar_node* pop_lowest_f(std::vector<t_astar_node*>& open_list);

int heuristic(t_astar_node nodeA, t_astar_node nodeB);

std::vector<t_astar_node*> get_neighbors(std::vector<std::vector<t_astar_node*>>& grid, t_astar_node* node);

void initialize_astar(std::vector<std::vector<t_astar_node*>>& grid, const Position& start, const Position& end, std::vector<t_astar_node*>& open_list);

bool is_end_node(t_astar_node* node, const Position& end);

std::vector<Position> reconstruct_path(t_astar_node* end_node);

void process_neighbors(std::vector<std::vector<t_astar_node*>>& grid, t_astar_node* current, const Position& end, std::vector<t_astar_node*>& open_list);

```

Utility methods for the A* pathfinding algorithm.

`create_astar_grid`, `delete_astar_grid`, `pop_lowest_f`, `heuristic`, `get_neighbors`, `initialize_astar`, `is_end_node`, `reconstruct_path`, `process_neighbors`


Associated Structure `t_astar_node`

Each node in A* search grid is represented by:

```c++
struct t_astar_node {
    Position pos;
    int g; // Cost from start
    int h; // Heuristic to goal
    int f; // Total cost = g + h
    bool visited;
    bool in_open_list;
    bool is_wall;
    t_astar_node* parent;
};
```

Structure representing a node in the [`A* algorithm`](./AStarAlgorithm.md) (position, costs, wall status, parent, etc.).

`Notes:`
The [`A* algorithm`](./AStarAlgorithm.md) allows the robot to avoid obstacles and find optimal paths.

The class manages collection, storage, sorting, and intelligent movement of the collector robot.