# Test1: Creating a Robot Class

We designed a small simulation program for waste collection.

We created a **Robot** class, which is inherited by two other classes: ***<em style="color: #949CDF;">CollectorRobot</em>*** and ***<em style="color: #949CDF;">SorterRobot</em>*** .

As their names suggest:

- **<em style="color: #949CDF;">CollectorRobot</em>** is a robot that navigates a map to collect waste and deposit it at a predefined destination.

- **<em style="color: #949CDF;">SorterRobot</em>** is a robot positioned at the CollectorRobot's destination. It receives the collected waste and performs sorting.
  
During program execution, the collection and sorting process will be visualized using <em style="color: #949CDF;">Ncurses</em>.


# Robot Abstract Class

The `Robot.hpp` header defines the abstract base class for [SorterRobot](SorterRobot.md) as well as supporting types for waste management and robot state.

---

## Enumerations

- **WasteType**: Enumerates the types of waste the robot can handle.
  - `PLASTIC`, `METAL`, `GLASS`, `ORGANIC`
- **status**: Represents the robot's operational state.
  - `INACTIVE`, `ACTIVE`

---

## Structs

### `Position`
Represents a 3D position with integer coordinates (`x`, `y`, `z`).  
Includes a `print()` method to display the position.

### `Waste`
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

- **Constructor**: 
```c++
Robot(const std::string& name);
```
  - Initializes the robot with a given name.

- **Pure Virtual Method**:
```c++
virtual void move(int dx, int dy, int dz) = 0;
```
  - Forces derived classes to implement movement logic.

- **Common Methods**:
```c++
void recharge(float amount);
```
  - Recharge the robot's battery.

```c++
void update_temperature();
```
  - Simulate temperature changes (e.g., overheating).

```c++
void displayStatus() const;
```
  - Print the robot's current status.

```c++
void stop();
```
  - Stop the robot.

- **Getters**:  
  Access the robot's name, battery level, position, status, and temperature.

- **Setters**:  
  Modify the robot's battery level, position, status, speed, and weight.

- **Destructor**:
  
```c++
virtual ~Robot()
```  
  - Ensures proper cleanup in derived classes.

### Protected Attributes

- `_name`: Robot's name (std::string).
- `_battery_level`: Current battery level (float).
- `_position`: Current position [struct Position](#position).
- `_status`: Current status [status](#status).
- `_speed`: Current speed (float).
- `_max_speed`: Maximum speed (float).
- `_weight`: Robot's weight (float).
- `_temperature`: Current temperature (float).