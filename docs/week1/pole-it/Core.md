# Core Class

The `Core` class is responsible for managing the main simulation loop, coordinating the actions of the robots, handling synchronization between threads, and updating the arena state. It serves as the central controller for the Epibot simulation, ensuring that the collector and sorter robots operate in harmony and that the simulation progresses smoothly.

```c++
class Core {
    public:
        Core(int ac, char **av);
        void simulate();
        ~Core();

    private:
        std::vector<std::string> _map;
        CollectorRobot *_collector;
        SorterRobot *_sorter;
        Ncurses *ncurses;
        std::vector<Waste> _sorterContainer;
        std::mutex _mutex;
        std::condition_variable _cv;
        void sorterRobotExecute();
        void CollectorRobotExecute();
        void drawAndWait();
        bool moveAlongPath(const std::vector<Position>& path, bool collect);
        size_t collectWastes(const std::vector<Position>& wastes, size_t startIndex);
        void moveToSorterRobot();
        bool _collectingFinished;
};
```

## Core Class Attributes

| Name                | Type                           | Description                                              |
|---------------------|--------------------------------|----------------------------------------------------------|
| `_map`              | `std::vector<std::string>`     | The arena map, each string is a row of the environment.  |
| `_collector`        | `CollectorRobot*`              | Pointer to the collector robot.                          |
| `_sorter`           | `SorterRobot*`                 | Pointer to the sorter robot.                             |
| `ncurses`           | `Ncurses*`                     | Pointer to the Ncurses UI handler.                       |
| `_sorterContainer`  | `std::vector<Waste>`           | Container for sorted waste items.                        |
| `_mutex`            | `std::mutex`                   | Mutex for thread synchronization.                        |
| `_cv`               | `std::condition_variable`      | Condition variable for thread coordination.              |
| `_collectingFinished` | `bool`                       | True if waste collection is finished.                    |

---

## Core Class Methods

| Name                                      | Return Type         | Description                                                        |
|-------------------------------------------|---------------------|--------------------------------------------------------------------|
| `Core(int ac, char **av)`                 | Constructor         | Initializes the simulation core with command-line arguments.        |
| `void simulate()`                         | `void`              | Runs the main simulation loop.                                      |
| `~Core()`                                 | Destructor          | Cleans up resources and shuts down the simulation.                  |
| `void sorterRobotExecute()`               | `void`              | Executes the logic for the sorter robot.                            |
| `void CollectorRobotExecute()`            | `void`              | Executes the logic for the collector robot.                         |
| `void drawAndWait()`                      | `void`              | Renders the simulation state and waits for input or events.         |
| `bool moveAlongPath(const std::vector<Position>& path, bool collect)` | `bool` | Moves a robot along a path, optionally collecting waste.            |
| `size_t collectWastes(const std::vector<Position>& wastes, size_t startIndex)` | `size_t` | Collects waste from specified positions, starting at `startIndex`.  |
| `void moveToSorterRobot()`                | `void`              | Moves the collector robot to the sorter robot's position.           |

---

## Deed Explanation of methods

- [Constructor](#constructor)
- [Public Methods](#public-methods)
- [Private Methods](#private-methods)
- [Destructor](#destructor)

---


## Constructor
```c++
Core::Core(int ac, char **av)
{
    Parser parser(ac, av[1]);
    _map = parser.parseFile();
    ncurses = new Ncurses(_map);
    _collector = new CollectorRobot("RTX5", 3, {1, 1, 0}, _map);
    _collector->setPosition({1, 1, 0}); // Position initiale
    _collector->setStatus(ACTIVE);
    _collectingFinished = false;
    _map[_collector->getPosition().y][_collector->getPosition().x] = 'R';
    _sorter = new SorterRobot("SorterBot");
    _sorter->setPosition({1, 1, 0});
    _sorter->setDestContainerPosition(PLASTIC, {1, 0, 0});
    _sorter->setDestContainerPosition(METAL, {2, 0, 0});
    _sorter->setDestContainerPosition(GLASS, {3, 0, 0});
    _sorter->setDestContainerPosition(ORGANIC, {4, 0, 0});
    _sorter->setStatus(ACTIVE);
}
```
---

<div style="max-width:500px; margin:40px auto; border-radius:16px; overflow:hidden; box-shadow:0 10px 20px rgba(0,0,0,0.15); transition:transform 0.3s ease;" onmouseover="this.style.transform='scale(1.02)'" onmouseout="this.style.transform='scale(1)'">
  <img src="/images/work_session_pole_it/jordan.jpeg" alt="Patrice DAGBE" style="width:100%; height:auto; display:block;">
</div>

---

### `Core(int ac, char **av)`

Initializes the simulation core by:

- Parsing the input map using `Parser`
- Creating and positioning the collector robot (`CollectorRobot`)
- Initializing the sorter robot (`SorterRobot`) and setting target container positions
- Setting up the ncurses interface

---

## Public Methods

### `void simulate()`
```cpp
void Core::simulate()
{
    std::thread collectorThread([this]() { this->CollectorRobotExecute(); });
    std::thread sorterThread([this]() { this->sorterRobotExecute(); });

    collectorThread.join();
    sorterThread.join();
}
```
Starts the simulation by spawning two threads:
- **Collector robot thread**: Handles waste collection.
- **Sorter robot thread**: Handles waste sorting.

Both threads are joined to ensure clean shutdown at the end of the simulation.

---

## Private Methods

### `void drawAndWait()`
```cpp
void Core::drawAndWait()
{
    ncurses->drawMap(_map);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
```
Draws the current state of the map and pauses for 200 milliseconds to animate the robots’ movement.

---

### `bool moveAlongPath(const std::vector<Position>& path, bool collect)`
```cpp
bool Core::moveAlongPath(const std::vector<Position>& path, bool collect)
{
    Position currentPos = _collector->getPosition();

    for (const auto& step : path) {
        int dx = step.x - currentPos.x;
        int dy = step.y - currentPos.y;

        _map[currentPos.y][currentPos.x] = ' ';
        _collector->move(dx, dy, 0);
        currentPos = _collector->getPosition();

        if (collect) {
            char c = _map[currentPos.y][currentPos.x];
            _collector->collect(c, currentPos);
        }

        _map[currentPos.y][currentPos.x] = 'R';
        ncurses->drawMap(_map);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return true;
}
```
Moves the collector robot along a predefined path.

- If `collect` is `true`, the robot will collect waste on the way.
- Updates the map and visual representation via `ncurses` -> [Ncurses](Ncurses.md).

---

### `size_t collectWastes(const std::vector<Position>& wastes, size_t startIndex)`
```cpp
size_t Core::collectWastes(const std::vector<Position>& wastes, size_t startIndex)
{
    size_t collected = startIndex;

    for (; collected < wastes.size(); collected++) {
        if (_collector->getContainer().size() >= (size_t)_collector->getMaxCapacity())
            break;

        Position target = wastes[collected];
        auto path = _collector->a_star(target.y, target.x);

        if (!moveAlongPath(path, true))
            break;

        if (_collector->getContainer().size() >= (size_t)_collector->getMaxCapacity())
            break;
    }

    return collected;
}
```
Attempts to collect wastes starting from a specified index in the list.

- Stops if the collector’s container is full or movement fails.
- Returns the next index to continue from.

---

### `void moveToSorterRobot()`
```cpp
void Core::moveToSorterRobot()
{
    auto toSorter = _collector->a_star(
        _collector->getSorterRobotPosition().y,
        _collector->getSorterRobotPosition().x
    );

    if (!moveAlongPath(toSorter, false))
        return;
}
```
Calculates a path from the collector’s current position to the sorter robot, and moves along that path.

---

### `void sorterRobotExecute()`
```cpp
void Core::sorterRobotExecute()
{
    while (ncurses->isRunning()) {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] {
            return !_sorterContainer.empty() || _collectingFinished || !ncurses->isRunning();
        });

        if (!ncurses->isRunning())
            break;

        if (_sorterContainer.empty()) {
            if (_collectingFinished) {
                _cv.notify_one();
                break;
            }
            continue;
        }
        auto wastesToSort = _sorterContainer;
        _sorter->setSrcContainer(std::make_pair(Position{2, 2, 0}, _sorterContainer));
        _sorterContainer.clear();
        lock.unlock();

        for (auto& waste : wastesToSort) {
            ncurses->setContainer(wastesToSort);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            _sorter->pickWaste(waste);
            _sorter->sort(waste);
            auto holdingWaste = _sorter->getHoldingWaste();
            _sorter->move(holdingWaste->position.x, holdingWaste->position.y, holdingWaste->position.z);
            ncurses->setPlasticWaste(_sorter->getContainer(PLASTIC));
            ncurses->setMetalWaste(_sorter->getContainer(METAL));
            ncurses->setGlassWaste(_sorter->getContainer(GLASS));
            ncurses->setOrganic(_sorter->getContainer(ORGANIC));
            ncurses->setContainer(_sorter->getSrcContainer());
        }
    }
}
```
Thread function for the sorter robot.

- Waits for the collector to deposit waste.
- Picks up each waste item, sorts it into the appropriate container based on its type.
- Updates `ncurses` display with sorted items.

---

### `void CollectorRobotExecute()`
```cpp
void Core::CollectorRobotExecute()
{
    while (ncurses->isRunning()) {
        drawAndWait();
        auto wastes = _collector->getWastePositions();

        if (wastes.empty() || _collector->getStatus() == INACTIVE) {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            ncurses->drawMap(_map);
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _collectingFinished = true;
                _cv.notify_one();
            }
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this] { return _sorterContainer.empty(); });
            }
            ncurses->drawMap(_map);
            ncurses->setisRunning(false);
            break;
        }
        _collector->sortWastesByDistance(_collector->getPosition(), wastes);
        
        size_t collected = 0;
        while (collected < wastes.size()) {
            collected = collectWastes(wastes, collected);
            
            moveToSorterRobot();
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _collector->decharge(_sorterContainer);
                _cv.notify_one();
            }
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    endwin();
    return;
}
```
Thread function for the collector robot.

- Continuously scans for wastes.
- Collects them and brings them to the sorter robot.
- Waits for the sorting process to finish before terminating the simulation.

---

## Destructor

### `~Core()`
```cpp
Core::~Core()
{
    if (ncurses)
        delete ncurses;
    if (_collector)
        delete _collector;
    if (_sorter)
        delete _sorter;
}
```
Cleans up dynamically allocated resources:
- Deletes `ncurses` interface
- Deletes the collector robot instance
- Deletes the sorter robot instance


---

## Notes

- Uses `std::mutex` and `std::condition_variable` for thread synchronization between the collector and sorter.
- Simulates autonomous waste management in a 2D environment.
- Relies on A* algorithm for pathfinding.
- Waste types handled: `PLASTIC`, `METAL`, `GLASS`, `ORGANIC`.