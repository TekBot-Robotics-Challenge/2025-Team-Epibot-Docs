# Getting Started

This guide will help you set up, build, and run the Waste Collection Simulation project.

## Prerequisites

Make sure you have the following installed on your system:

- **`C++`**
- **`g++`** (C++ compiler)
- **`make`**
- **`ncurses`** library

On Debian/Ubuntu-based systems, you can install the required dependencies using:

```bash
sudo apt update
sudo apt install g++ make libncurses5-dev libncursesw5-dev
```

## 📁 Project Structure

Here’s what the project directory looks like:

```bash
week1/IT Pole/
├── Environment/
│   └── map.txt
├── include/
│   ├── CollectorRobot.hpp
│   ├── Core.hpp
│   ├── Error.hpp
│   ├── ncurses.hpp
│   ├── Parser.hpp
│   ├── Robot.hpp
│   └── SorterRobot.hpp
├── main.cpp
├── Makefile
└── src/
    ├── CollectorRobot.cpp
    ├── Core.cpp
    ├── Ncurses.cpp
    ├── Parser.cpp
    ├── Robot.cpp
    └── SorterRobot.cpp
```

## How to Build and Run

1. Clone (if you have access) and navigate into the repository:
   
```bash
git clone git@github.com:TekBot-Robotics-Challenge/2025-Team-Epibot-Code.git
cd week1/IT\ Pole/
```

2. Build the project:

```bash
make
```

This will compile all .cpp files in src/ using the headers in include/ and output an executable.

3. Run the simulation:

```bash
./robot Environment/map.txt
```

Replace `Environment/map.txt` with the path to your custom map file if needed.

## `Notes`:

- Ensure the map file exists and is formatted correctly.

- The terminal window should be large enough for proper Ncurses rendering.

- To stop the simulation, press `Q` or `Ctrl+C`.

If you experience any issues, consult the source files or refer to the Project Demo page for more context.
