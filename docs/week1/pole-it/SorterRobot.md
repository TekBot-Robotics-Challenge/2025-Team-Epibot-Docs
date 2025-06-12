# SorterRobot Class

The `SorterRobot` class inherits from the abstract `Robot` class and represents a robot specialized in sorting waste. It manages sorting and depositing different types of waste in specific locations.

<div style="max-width:500px; margin:40px auto; border-radius:16px; overflow:hidden; box-shadow:0 10px 20px rgba(0,0,0,0.15); transition:transform 0.3s ease;" onmouseover="this.style.transform='scale(1.02)'" onmouseout="this.style.transform='scale(1)'">
  <img src="/images/work_session_pole_it/maite.jpeg" alt="Patrice DAGBE" style="width:100%; height:auto; display:block;">
</div>

```c++
class SorterRobot : public Robot {
    public:
        SorterRobot(const std::string& name);

        SorterRobot(const std::string& name, float weight, float speed, float max_speed);

        void move(int dx, int dy, int dz) override;

        void sort(Waste waste);

        std::vector<Waste> getSrcContainer(void) const;
        
        Waste * getHoldingWaste(void) const;

        void setSrcContainer(std::pair<Position, std::vector<Waste>> srcContainer);
        
        void setDestContainerPosition(WasteType type, Position position);

        void pickWaste(Waste &waste);

        std::vector<Waste> getContainer(WasteType _type) const;

        ~SorterRobot();

    private:
        Waste *holdingWaste;
        std::pair<Position, std::vector<Waste>> _srcContainer;
        std::map<WasteType, std::pair<Position, std::vector<Waste>>> _destContainers;
};
```

## Attributes
```c++
//Pointer on the actually holded waste by the robot
Waste *holdingWaste;
```

```c++
//Container of wastes to sort and its position
std::pair<Position, std::vector<Waste>>;
```

```c++
//Containers for sorted waste and their positions identified by the type of waste they contain
std::map<WasteType, std::pair<Position, std::vector<Waste>>> _destContainers;
```


## Methods

### Constructors
```c++
SorterRobot::SorterRobot(const std::string& name) : Robot(name)
{
    holdingWaste = nullptr;
    _srcContainer = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[PLASTIC] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[METAL] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[GLASS] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[ORGANIC] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
}
```
  - Initializes the sorting robot attributes (`name` and waste containers).
---

```c++
SorterRobot::SorterRobot(const std::string& name, float weight, float speed, float max_speed)
: Robot(name, weight, speed, max_speed)
{
    holdingWaste = nullptr;
    _srcContainer = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[PLASTIC] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[METAL] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[GLASS] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());
    _destContainers[ORGANIC] = std::make_pair(Position{0, 0, 0}, std::vector<Waste>());   
}
```
  - Second alternative to initialize the sorting robot attributes (`name`, `weight`, `speed` `max_speed` and waste containers).
---

### Destructor
```c++
SorterRobot::~SorterRobot()
{
    //Clears the source container's waste list
    _srcContainer.second.clear();

    //Iterates over each destination container and clears their contents.
    for (auto& pair : _destContainers) {
        pair.second.second.clear();
    }
    
    //Ensures that the robot no longer holds any waste.
    holdingWaste = nullptr;
    
    //Clears the map of destination containers
    _destContainers.clear();
}
```
  - This destructor ensures that the robot releases all structures it handled during its life cycle, even.

### Movement
```c++
void SorterRobot::move(int dx, int dy, int dz)
{
    try {
        if (holdingWaste == nullptr) {
        throw Error("Error: No waste is being held to move.");
        return;
    }
    while (_position.x != dx || _position.y != dy || _position.z != dz) {

        //Simulate movement towards the target position
        if (_position.x < dx) _position.x++;
        else if (_position.x > dx) _position.x--;
        if (_position.y < dy) _position.y++;
        else if (_position.y > dy) _position.y--;
        if (_position.z < dz) _position.z++;
        else if (_position.z > dz) _position.z--;

        //Decrease battery level after movement
        _battery_level -= 1.0;
        if (_battery_level < 0) {
            throw Error("Error: Battery level too low to move.");
            _status = INACTIVE;
            return;
        }

        //Update temperature after movement
        update_temperature();
        if (_temperature >= 50.0) {
            throw Error("Error: Robot overheated.");
            _status = INACTIVE;
            return;
        }
    }

    //Set robot status to ACTIVE and release the held waste
    _status = ACTIVE;
    holdingWaste = nullptr;
    } catch (const Error& e) {
        std::cerr << "Error: " << e.what();
        return;
    }
    return;
}
```
  - Chek if the robot is holding a waste.
  - Moves the robot by the given coordinates.
  - Check battery level and update robot temperature
---
### Waste Management
```c++
void SorterRobot::pickWaste(Waste &waste)
{
    // Set the pointer to the waste being held
    holdingWaste = &waste;
    try {
        // Try to find the waste in the source container
        auto& srcWastes = _srcContainer.second;
        auto it = std::find(srcWastes.begin(), srcWastes.end(), waste);
        if (it != srcWastes.end()) {
            // Remove the waste from the source container if found
            srcWastes.erase(it);
        } else {
            // Throw an error if the waste is not found
            throw Error("Waste not found in source container.");
        }
    } catch (const Error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return;
}
```
  - Set a new `holdingWaste`
  - Drop the held waste by the robot from the source container

```c++
void SorterRobot::sort(Waste waste)
{
    try {
        // Check if the robot is holding a waste before sorting
        if (holdingWaste == nullptr)
            throw Error("No waste is being held to sort.");
        // Find the appropriate destination container for the waste type
        auto it = _destContainers.find(waste.type);
        if (it != _destContainers.end()) {
            // Update the waste position to the container's position
            holdingWaste->position = it->second.first;
            // Add the waste to the destination container
            it->second.second.push_back(*holdingWaste);
        } else {
            // Throw an error if the waste type is not found
            throw Error("Waste type not found in destination containers.");
        }
    } catch (const Error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
```
  - Find the appropriate container (`plastic`, `metal`, `organic` or `glass`) for the collected waste and add it there
---

### Getters and Setters
```c++
std::vector<Waste> SorterRobot::getSrcContainer(void) const
{
    return _srcContainer.second;
}
```
  - return the container of wastes to sort

```c++
std::vector<Waste> SorterRobot::getContainer(WasteType _type) const
{
   try {
        auto it = _destContainers.find(_type);
        if (it != _destContainers.end())
            return it->second.second;
        throw Error("Waste type not found in destination containers.");
   } catch (const Error& e) {
       std::cerr << "Error: " << e.what() << std::endl;
   }
   return std::vector<Waste>();
}
```
  - return the container corresponding to the specified type

```c++
void SorterRobot::setDestContainerPosition(WasteType type, Position position)
{
    try {
        auto it = _destContainers.find(type);
        if (it != _destContainers.end()) {
            it->second.first = position;
        } else
            throw Error("Waste type not found in destination containers.");
    } catch (const Error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return;
}
```
  - set a sorted waste container by the `type` and the `position`

```c++
void SorterRobot::setSrcContainer(std::pair<Position, std::vector<Waste>> srcContainer)
{
    _srcContainer = srcContainer;
    return;
}
```
  - set the wastes to sort from the decharge of the CollectorRobot