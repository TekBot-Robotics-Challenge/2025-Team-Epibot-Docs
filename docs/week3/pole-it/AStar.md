# AStar

## Overview

The `AStar` class implements the A* pathfinding algorithm for the grid-based map in the Tekbot navigation stack.  
It operates on a ROS2 occupancy grid, builds a search graph, and computes the shortest path between two positions, considering obstacles.

---

### Data Structures

#### Position

```c++
typedef struct pos {
    int x;
    int y;
    int z;
    void print() const {
        std::cout << "(" << x << ", " << y << ", " << z << ")";
    }
} Position;
```
- **Purpose:** Represents a 3D position in the grid.

#### t_astar_node

```c++
typedef struct s_astar_node {
    //node position in the grid
    Position pos;
    //cost from start to this node
    int g;
    //heuristic cost to goal
    int h;
    //total cost (g + h)
    int f;
    //true if this node is an obstacle
    bool is_wall;
    //true if node has been visited
    bool visited;
    //true if node is in the open list
    bool in_open_list;
    //pointer to parent node for path reconstruction
    struct s_astar_node *parent;
} t_astar_node;
```
- **Purpose:** Stores all information needed for A* search at each grid cell.
---

## Class: `AStar`
```c++
class AStar
{
    public:
    nav_msgs::msg::OccupancyGrid _map;

    AStar(const nav_msgs::msg::OccupancyGrid &map);
    ~AStar(){};

    std::vector<std::vector<t_astar_node*>> create_astar_grid(int height, int width, const Position& goal);
    void delete_astar_grid(std::vector<std::vector<t_astar_node*>>& grid);
    t_astar_node* pop_lowest_f(std::vector<t_astar_node*>& open_list);
    int heuristic(t_astar_node nodeA, t_astar_node nodeB);
    std::vector<t_astar_node*> get_neighbors(std::vector<std::vector<t_astar_node*>>& grid, t_astar_node* node);
    void initialize_astar(std::vector<std::vector<t_astar_node*>>& grid, const Position& start, const Position& end, std::vector<t_astar_node*>& open_list);
    bool is_end_node(t_astar_node* node, const Position& end);
    std::vector<Position> reconstruct_path(t_astar_node* end_node);
    void process_neighbors(std::vector<std::vector<t_astar_node*>>& grid, t_astar_node* current, const Position& end, std::vector<t_astar_node*>& open_list);
};
```
  - Definition of the AStar class for grid-based pathfinding.

### Attributes

- **_map**:  
  `nav_msgs::msg::OccupancyGrid`  
  Stores the occupancy grid map used for pathfinding.

---

### Methods

#### Constructor

```c++
AStar::AStar(const nav_msgs::msg::OccupancyGrid &map)
{
    _map = map;
}
```
- **Purpose:** Initializes the AStar object with the provided occupancy grid map.

---

#### Grid Creation

```c++
std::vector<std::vector<t_astar_node*>> AStar::create_astar_grid(int height, int width, const Position& goal)
{
    std::vector<std::vector<t_astar_node*>> grid(height, std::vector<t_astar_node*>(width, nullptr));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            t_astar_node* node = new t_astar_node;
            node->pos.y = y;
            node->pos.x = x;
            node->pos.z = 0;
            node->g = INT_MAX;
            node->h = 0;
            node->f = INT_MAX;
            node->visited = false;
            node->in_open_list = false;
            node->parent = nullptr;

            int index = y * width + x;
            int8_t value = _map.data[index];
            //mark as wall if cell is occupied
            node->is_wall = (value != 0);

            grid[y][x] = node;
        }
    }

    return grid;
}
```
- **Purpose:** Builds a 2D grid of nodes representing the map, marking obstacles as walls.

---

#### Grid Deletion

```c++
void AStar::delete_astar_grid(std::vector<std::vector<t_astar_node*>>& grid)
{
    for (auto& row : grid) {
        for (auto& node : row) {
            delete node;
        }
    }
    grid.clear();
}
```
- **Purpose:** Frees memory allocated for the grid after pathfinding.

---

#### Pop Node with Lowest Cost

```c++
t_astar_node* AStar::pop_lowest_f(std::vector<t_astar_node*>& open_list)
{
    if (open_list.empty())
        return nullptr;

    auto min_it = open_list.begin();
    for (auto it = open_list.begin() + 1; it != open_list.end(); ++it) {
        if ((*it)->f < (*min_it)->f)
            min_it = it;
    }

    t_astar_node* result = *min_it;
    open_list.erase(min_it);
    return result;
}
```
- **Purpose:** Selects and removes the node with the lowest total cost (f) from the open list.

---

#### Heuristic Function

```c++
int AStar::heuristic(t_astar_node nodeA, t_astar_node nodeB)
{
    int dx = abs(nodeB.pos.x - nodeA.pos.x);
    int dy = abs(nodeB.pos.y - nodeA.pos.y);

    return std::max(dx, dy);
}
```
- **Purpose:** Estimates the cost from a node to the goal (used by A*).

---

#### Neighbor Retrieval

```c++
std::vector<t_astar_node*> AStar::get_neighbors(std::vector<std::vector<t_astar_node*>>& grid, t_astar_node* node)
{
    int dx[] = {  0, -1, -1, -1,  0,  1,  1,  1 };
    int dy[] = { -1, -1,  0,  1,  1,  1,  0, -1 };

    int width = _map.info.width;
    int height = _map.info.height;

    std::vector<t_astar_node*> neighbors;
    for (int i = 0; i < 8; ++i) {
        int nx = node->pos.x + dx[i];
        int ny = node->pos.y + dy[i];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height)
            continue;
        neighbors.push_back(grid[ny][nx]);
    }
    return neighbors;
}
```
- **Purpose:** Returns all valid (in-bounds) neighbors of a node, including diagonals.

---

#### A* Initialization

```c++
void AStar::initialize_astar(std::vector<std::vector<t_astar_node*>>& grid, const Position& start, const Position& end, std::vector<t_astar_node*>& open_list)
{
    t_astar_node* start_node = grid[start.y][start.x];
    t_astar_node* end_node = grid[end.y][end.x];

    start_node->g = 0;
    start_node->h = heuristic(*start_node, *end_node);
    start_node->f = start_node->g + start_node->h;
    open_list.push_back(start_node);
    start_node->in_open_list = true;
}
```
- **Purpose:** Prepares the grid and open list for the A* search, initializing the start node.

---

#### Goal Check

```c++
bool AStar::is_end_node(t_astar_node* node, const Position& end)
{
    return node->pos.x == end.x && node->pos.y == end.y;
}
```
- **Purpose:** Checks if the current node is the goal node.

---

#### Path Reconstruction

```c++
std::vector<Position> AStar::reconstruct_path(t_astar_node* end_node)
{
    std::vector<Position> path;
    while (end_node != nullptr) {
        path.push_back(end_node->pos);
        end_node = end_node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
```
- **Purpose:** Builds the path from start to goal by following parent pointers.

---

#### Neighbor Processing

```c++
void AStar::process_neighbors(std::vector<std::vector<t_astar_node*>>& grid, t_astar_node* current, const Position& end, std::vector<t_astar_node*>& open_list)
{
    t_astar_node* end_node = grid[end.y][end.x];
    std::vector<t_astar_node*> neighbors = get_neighbors(grid, current);

    for (t_astar_node* neighbor : neighbors) {
        if (neighbor->is_wall || neighbor->visited)
            continue;

        int tentative_g = current->g + 1;

        if (tentative_g < neighbor->g) {
            neighbor->parent = current;
            neighbor->g = tentative_g;
            neighbor->h = heuristic(*neighbor, *end_node);
            neighbor->f = neighbor->g + neighbor->h;

            if (!neighbor->in_open_list) {
                open_list.push_back(neighbor);
                neighbor->in_open_list = true;
            }
        }
    }
}
```
- **Purpose:** Updates neighbors' costs and parent pointers, adding them to the open list if needed.

---