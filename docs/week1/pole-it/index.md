# Test1: Creating a Robot Class

We designed a small simulation program for waste collection.

We created a **Robot** class, which is inherited by two other classes: [CollectorRobot](CollectorRobot.md) and [SorterRobot](SorterRobot.md).

As their names suggest:

- **<em style="color: #949CDF;">CollectorRobot</em>** is a robot that navigates a map to collect waste and deposit it at a predefined destination.

- **<em style="color: #949CDF;">SorterRobot</em>** is a robot positioned at the CollectorRobot's destination. It receives the collected waste and performs sorting.
  
During program execution, the collection and sorting process will be visualized using <em style="color: #949CDF;">Ncurses</em>.


# Robot Abstract Class

The `Robot.hpp` header defines the abstract base class for [SorterRobot](SorterRobot.md) as well as supporting types for waste management and robot state.

---

## Enumerations
### WasteType
```c++
enum WasteType {
    PLASTIC,
    METAL,
    GLASS,
    ORGANIC
};
```
Enumerates the types of waste the robot can handle: `PLASTIC`, `METAL`, `GLASS`, `ORGANIC`

### status
```c++
enum status {
    INACTIVE,
    ACTIVE
};
```
Represents the robot's operational state: `INACTIVE`, `ACTIVE`

---

## Structures
### `Position`
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
Represents a 3D position with integer coordinates (`x`, `y`, `z`).  
Includes a `print()` method to display the position.

### `Waste`
```c++
typedef struct waste_s {
    char character;
    WasteType type;
    Position position;
    bool operator==(const waste_s& other) const {
        return character == other.character &&
               type == other.type &&
               position.x == other.position.x &&
               position.y == other.position.y &&
               position.z == other.position.z;
    }
} Waste;
```
Represents a waste item with:
- `character`: a char symbolizing the waste
- `type`: the type of waste (`WasteType`)
- `position`: the waste's position (`Position`)
- `operator==`: allows comparison between two `Waste` objects (useful for searching in containers)

---

## Class `Robot`

An **abstract base class** (cannot be instantiated directly) representing a generic robot.

```c++
class Robot {
    public:
        // Constructor
        Robot(const std::string& name);
        
        // [============== Pure Virtual Method ============]
        virtual void move(int dx, int dy, int dz) = 0;

        // [============== Common Methods ============]
        void recharge(float amount);
        void update_temperature(); // Simulation of overgheadting
        void displayStatus() const;
        void stop();

        // [============== Getters ============]
        std::string getName() const;
        float getBatteryLevel() const;
        Position getPosition() const;
        status getStatus() const;
        float getTemperature() const;

        // [============== Setters ============]
        void setBatteryLevel(float battery_level);
        void setPosition(Position position);
        void setStatus(status status);
        void setSpeed(float new_speed);
        void setWeight(float weight);
        
        // Destructor
        virtual ~Robot();
        
    protected:
        std::string _name;
        float _battery_level;
        Position _position;
        status _status;
        float _speed;
        float _max_speed;
        float _weight;
        float _temperature;
};
```
---

### Main Features

### Protected Attributes

```c++
//Robot's name
std::string _name;
//Current battery level
float _battery_level;
//Current position
Position _position;
//Current status
status _status;
//Current speed
float _speed;
//Maximum speed
float _max_speed;
//Robot's weight
float _weight;
//Current temperature
float _temperature;
```
---

### Constructor 
```c++
Robot::Robot(const std::string& name)
    : _name(name), _battery_level(100.0), _position({0,0,0}), _status(INACTIVE),
      _speed(1.0), _max_speed(5.0), _weight(10.0), _temperature(0.0)
{}
```
  - Initializes the robot with a given name and default values for attributes.
---

### Pure Virtual Method
```c++
virtual void move(int dx, int dy, int dz) = 0;
```
  - Forces derived classes to implement movement logic.
---

## Common Methods

```c++
void Robot::recharge(float amount) {
    _battery_level += amount;
    if (_battery_level > 100.0f)
        _battery_level = 100.0f;
}
```
  - Recharge the robot's battery.

```c++
void Robot::update_temperature() {
    _temperature += 1.0f;
    if (_temperature > 100.0f)
        _temperature = 100.0f;
}
```
  - Simulate temperature changes.

```c++
void Robot::displayStatus() const {
    std::cout << "Robot: " << _name << std::endl;
    std::cout << "Status: " << (_status == ACTIVE ? "ACTIVE" : "INACTIVE") << std::endl;
    std::cout << "Battery: " << _battery_level << "%" << std::endl;
    std::cout << "Temperature: " << _temperature << "°C" << std::endl;
    std::cout << "Position: ";
    _position.print();
    std::cout << std::endl;
}
```
  - Display the robot's current status.

```c++
void Robot::stop() {
    _status = INACTIVE;
    _speed = 0.0f;
}
```
  - Stop the robot.
---

### Getters

```c++
std::string Robot::getName() const {
    return _name;
}

float Robot::getBatteryLevel() const {
    return _battery_level;
}

Position Robot::getPosition() const {
    return _position;
}

status Robot::getStatus() const {
    return _status;
}

float Robot::getTemperature() const {
    return _temperature;
}
```
  - Access the robot's name, battery level, position, status, and temperature.
---

### Setters

```c++
void Robot::setBatteryLevel(float battery_level) {
    _battery_level = battery_level;
}

void Robot::setPosition(Position position) {
    _position = position;
}

void Robot::setStatus(status status_) {
    _status = status_;
}

void Robot::setSpeed(float new_speed) {
    if (new_speed > _max_speed)
        _speed = _max_speed;
    else if (new_speed < 0.0f)
        _speed = 0.0f;
    else
        _speed = new_speed;
}

void Robot::setWeight(float weight) {
    _weight = weight;
}
```
  - Modify the robot's battery level, position, status, speed, and weight.
---

### Destructor

```c++
Robot::~Robot() {}
```
  - Ensures proper cleanup in derived classes.
