# SorterRobot Class

The `SorterRobot` class inherits from the abstract `Robot` class and represents a robot specialized in sorting waste. It manages sorting and depositing different types of waste in specific locations.

## Specific Attributes
- **holdingWaste**: Pointer on the actually holded waste by the robot
- **_srcContainer**: container of wastes to sort
- **_destContainers**: map of each waste type container for sorted wastes

## Specific Methods

### Constructor
```c++
SorterRobot(const std::string& name)
```
  - Initializes a sorting robot with the given name.

### Movement
```c++
void move(int dx, int dy, int dz) override
```
  - Moves the robot by the given coordinates.

### Waste Management
```c++
void pickWaste(Waste& waste)
```
  - Allows the robot to pick up a waste item at a given position.

```c++
void sort(Waste waste)
```
  - Sorts waste by type.

### Getters and Setters
```c++
std::vector<Waste> getSrcContainer(void) const
```
  - return the container of wastes to sort

```c++
std::vector<Waste> getContainer(WasteType _type) const
```
  - return the container corresponding to the specified type

```c++
void setDestContainerPosition(WasteType type, Position position)
```
  - set a sorted wastes container by the ***type*** and the ***position***

```c++
void setSrcContainer(std::pair<Position, std::vector<Waste>> srcContainer)
```
  - set the wastes to sort from the decharge of the CollectorRobot

### Execution
```c++
void execute(void)
```
  - launch the robot: call successively pickWaste, sort and move methods until all wastes are sorted