# Ncurses Class

The `Ncurses` class manages the game's graphical display in the terminal using the `ncurses` library. It allows us to create multiple windows, to display the arena map as well as information about the different types of waste collected, and to simulate the waste collection process.

<div style="max-width:500px; margin:40px auto; border-radius:16px; overflow:hidden; box-shadow:0 10px 20px rgba(0,0,0,0.15); transition:transform 0.3s ease;" onmouseover="this.style.transform='scale(1.02)'" onmouseout="this.style.transform='scale(1)'">
  <img src="/images/work_session_pole_it/together.jpeg" alt="Patrice DAGBE" style="width:100%; height:auto; display:block;">
</div>

## Class Definition
```C++

class Ncurses {
    public:
        Ncurses(std::vector<std::string> arenaMap);
        void drawMap(const std::vector<std::string> map);
        void setPlasticWaste(std::vector<Waste> waste) { plasticWaste = waste.size(); }
        void setMetalWaste(std::vector<Waste> waste) { metalWaste = waste.size(); }
        void setGlassWaste(std::vector<Waste> waste) { glassWaste = waste.size(); }
        void setOrganic(std::vector<Waste> waste) { organic = waste.size(); }
        void setContainer(std::vector<Waste> waste) { container = waste.size(); }
        int getPlasticWaste() const { return plasticWaste; }
        int getMetalWaste() const { return metalWaste; }
        int getGlassWaste() const { return glassWaste; }
        int getOrganic() const { return organic; }
        void setContainer(int waste) { container = waste; }
        int getContainer() const { return container; }
        void setisRunning(bool isRunning) {_isRunning = isRunning;}
        void setBackGround();
        void setColor_pair();
        void refreshWindows();
        void drawBorders();
        void Keypad_and_nodelay();
        void createWindows();
        void WasteCollect();
        bool isRunning() const {return _isRunning;}
        void destroyWindow();
        ~Ncurses();
    private:
        WINDOW *_window;
        WINDOW *_plasticWindow;
        WINDOW *_metalWindow;
        WINDOW *_glassWindow;
        WINDOW *_organic;
        WINDOW *_container;
        bool _isRunning;
        int plasticWaste;
        int metalWaste;
        int glassWaste;
        int organic;
        int container;
        int _width;
        int _height;
        int max_x;
        int max_y;
};

```

## Main Attributes of this Class
- `_window`: Main window displaying the arena map
- `_plasticWindow`: Window displaying the quantity of plastic waste
- `_metalWindow`: Window displaying the quantity of scrap metal
- `_glassWindow`: Window displaying the quantity of glass waste
- `_organic`: Window displaying the quantity of organic waste
- `_container`: Window displaying the number of containers
- plasticWaste, metalWaste, glassWaste, organic, container: Counters for each type of waste
- max_x, max_y: Terminal dimensions
- _height, _width: Map dimensions
- `_isRunning`: Display running status

## Main Methods
### Constructor & Destructor

```c++
Ncurses::Ncurses(std::vector<std::string> arenaMap)
{
    initscr(); // Initialize ncurses mode
    curs_set(0);
    start_color(); // Init color
    cbreak(); // Disable line buffering
    noecho(); // Don't echo input characters
    keypad(stdscr, TRUE); // Enable special keys (like arrow keys)
    getmaxyx(stdscr, max_y, max_x); // Get the size of the terminal window
    setColor_pair(); // Set color pairs for different elements
    _height = arenaMap.size();
    _width = arenaMap[0].size();
    plasticWaste = 0; // Initialize waste counters
    metalWaste = 0; // Initialize waste counters
    glassWaste = 0; // Initialize waste counters
    //householdWaste = 0; // Initialize waste countersorga
    organic = 0; // Initialize waste counters
    container = 0; // Initialize waste counters

    _isRunning = true; // Initialize running state
    createWindows(); // Create the windows for the game
    refresh(); // Refresh the standard screen
}

Ncurses::~Ncurses() {
    endwin(); 
}
```
### Ncurses()

- The `Ncurses::Ncurses()` constructor initializes an instance of the Ncurses class using a `std::vector<std::string> arenaMap`, which represents an arena map. This constructor allows the class to be configured with a predefined map upon creation.
- Initializes ncurses windows and garbage counters.

### ~Ncurses()
- Ends ncurses mode properly.
- The `Ncurses::~Ncurses()` destructor is responsible for cleaning up the Ncurses environment by calling the endwin()       function, which restores the terminal to its normal operating mode after Ncurses operations. This ensures proper terminal behavior when the Ncurses object is destroyed.

### Getter And Setter 

- `int getPlasticWaste() const`  
- `int getMetalWaste() const`  
- `int getGlassWaste() const`  
- `int getOrganic() const`  
- `int getContainer() const`  
  Returns the current count of the wasteContainer.
- `void setPlasticWaste(std::vector<Waste> waste)`  
- `void setMetalWaste(std::vector<Waste> waste)`  
- `void setGlassWaste(std::vector<Waste> waste)`  
- `void setOrganic(std::vector<Waste> waste)`  
- `void setContainer(std::vector<Waste> waste)` Sets the waste counter to the size of the given waste vector.
- `void setisRunning(bool isRunning)` Sets the running state of   the display.
- `bool isRunning() const` Returns the running state of the display.

### createWindows()
```c++
void Ncurses::createWindows()
{
    _window = newwin(_height + 2, _width + 2, 0, get_percent(max_x, 25)); // Create a new window
    _plasticWindow = newwin(3, 20, get_percent(max_y, 10), 0); // Create a plastic waste window
    _metalWindow = newwin(3, 20, get_percent(max_y, 25), 0); // Create a metal waste window
    _glassWindow = newwin(3, 20, get_percent(max_y, 40), 0); // Create a glass waste window
    _organic = newwin(3, 20, get_percent(max_y, 55), 0); // Create a household waste window
    _container = newwin(3, 20, get_percent(max_y, 70), 0); // Create a container window
    setBackGround(); // Set the background colors for the windows
    Keypad_and_nodelay(); // Enable keypad and non-blocking input for the windows
    drawBorders(); // Draw borders around the windows
}
```
- Creates and positions all necessary windows.
- The Ncurses::createWindows function is a member of the Ncurses class and is responsible for creating and initializing windows using the Ncurses library.
- It likely sets up the necessary visual components for terminal-based user interfaces.


### drawMap(std::vector<std::string> map)
```C++
void Ncurses::drawMap(std::vector<std::string> map)
{
    int ch = getch();
    if (ch == 'q' || ch == 'Q') {
        _isRunning = false;
        destroyWindow();
        endwin();
        exit(0);
    }
    for (size_t i = 0; i < map.size(); ++i) {
        std::string line = map[i];
        for (size_t j = 0; j < line.size(); ++j) {
            if (line[j] == '#') {
                wattron(_window, COLOR_PAIR(2)); // Wall color
            } else if (line[j] == '.') {
                wattron(_window, COLOR_PAIR(1)); // Empty space color
            } else if (line[j] == 'G') {
                wattron(_window, COLOR_PAIR(3)); // Goal color
            } else if (line[j] == 'P') {
                wattron(_window, COLOR_PAIR(4)); // Player color
            } else if (line[j] == 'M') {
                wattron(_window, COLOR_PAIR(5)); // Monster color
            } else if (line[j] == '0') {
                wattron(_window, COLOR_PAIR(7)); // Hero color
            } else if (line[j] == 'R') {
                wattron(_window, COLOR_PAIR(6)); // Robot color
            } else {
                wattroff(_window, COLOR_PAIR(1)); // Default color
            }
            mvwprintw(_window, i + 1, j + 1, "%c", line[j]);
        }
    }
    WasteCollect(); // Collect and display waste information
    refreshWindows(); // Refresh all windows to show the changes
    usleep(100000); // Sleep for 100 milliseconds to control the refresh rate
}
```
- Displays the arena map and handles keyboard input (e.g., quit with 'q').
- Display of waste counters
- The Ncurses::drawMap function renders a map represented as a vector of strings onto an ncurses window, applying different color pairs based on specific characters in the map (e.g., walls, goals, players). It also handles user input to terminate the program when 'q' or 'Q' is pressed, updates the display, and controls the refresh rate with a brief delay.

### destroyWindow()
```C++
void Ncurses::destroyWindow()
{
    werase(_window);
    werase(_plasticWindow);
    werase(_metalWindow);
    werase(_glassWindow);
    werase(_organic);
    werase(_container);
    erase();
    delwin(_window);
    delwin(_plasticWindow);
    delwin(_metalWindow);
    delwin(_glassWindow);
    delwin(_organic);
    delwin(_container);
}
```
- Clears and destroys all windows.
- Free the space allocate for windows

### Keypad_and_nodelay()
```C++
void Ncurses::Keypad_and_nodelay()
{
    nodelay(stdscr, TRUE); // Make the standard screen non-blocking
    nodelay(_window, TRUE); // Make the main window non-blocking
    nodelay(_plasticWindow, TRUE); // Make the plastic window non-blocking
    nodelay(_metalWindow, TRUE); // Make the metal window non-blocking
    nodelay(_glassWindow, TRUE); // Make the glass window non-blocking
    nodelay(_organic, TRUE); // Make the household waste window non-blocking
    nodelay(_container, TRUE); // Make the container window non-blocking
    keypad(_plasticWindow, TRUE); // Enable special keys in the plastic window
    keypad(_metalWindow, TRUE); // Enable special keys in the metal window
    keypad(_glassWindow, TRUE); // Enable special keys in the glass window
    keypad(_organic, TRUE); // Enable special keys in the household waste window
    keypad(_window, TRUE); // Enable special keys in the main window
    keypad(_container, TRUE); // Enable special keys in the container window
}
```
- Enables `non-blocking mode`.
- this enable the program to check for user input without waiting or pausing if no key is pressed. This allows the       program to keep running and updating the display even if the user doesn't press any keys.
- In summary, this makes the ncurses windows responsive to user input


## Utility 

### setBackGround()
```c++
void Ncurses::setBackGround()
{
    wbkgd(_window, COLOR_PAIR(1)); // Set the background color for the main window
    wbkgd(_plasticWindow, COLOR_PAIR(4)); // Set the background color for plastic waste window
    wbkgd(_metalWindow, COLOR_PAIR(5)); // Set the background color for metal waste window
    wbkgd(_glassWindow, COLOR_PAIR(6)); // Set the background color for glass waste window
    wbkgd(_organic, COLOR_PAIR(7)); // Set the background color for household waste window
    wbkgd(_container, COLOR_PAIR(2)); // Set the background color for container window
}
```
- Sets background colors for each window with the wbkg function in ncurses.

### drawBorders()
```c++
void Ncurses::drawBorders()
{
    box(_window, 0, 0); // Draw a border around the main window
    box(_plasticWindow, 0, 0); // Draw a border around the plastic waste window
    box(_metalWindow, 0, 0); // Draw a border around the metal waste window
    box(_glassWindow, 0, 0); // Draw a border around the glass waste window
    box(_organic, 0, 0); // Draw a border around the household waste window
    box(_container, 0, 0); // Draw a border around the container window
}
```
- Draws borders around each window.
- The Ncurses::drawBorders  is responsible for rendering borders on the terminal interface
- we use box ncurses function 

### refreshWindows()
```C++
void Ncurses::refreshWindows()
{
    wrefresh(_window);
    wrefresh(_plasticWindow);
    wrefresh(_metalWindow);
    wrefresh(_glassWindow);
    wrefresh(_organic);
    wrefresh(_container);
    refresh();
}
```
- Refreshes all windows to display changes.
- We use refresh() and wrefresh() for that 

### WasteCollect()
```C++
void Ncurses::WasteCollect()
{
    mvwprintw(_plasticWindow, 1, 1, "Plastic Waste: %d", plasticWaste);
    mvwprintw(_metalWindow, 1, 1, "Metal Waste: %d", metalWaste);
    mvwprintw(_glassWindow, 1, 1, "Glass Waste: %d", glassWaste);
    mvwprintw(_organic, 1, 1, "Orgnaic : %d", organic);
    mvwprintw(_container, 1, 1, "Container: %d", container);
}
```
- Displays the quantities of each type of waste in the dedicated windows.
- For to print things on curses window we use 


void setColor_pair()
- Initializes the color pairs used in the display.


```C++
int get_percent(int a, int p)
{
    int s = static_cast<int>((static_cast<float>(a) / 100) * p);
    return s;
}
```
- Calculates a percentage of the terminal size to position the windows.