<h1 style="font-size: 24px; font-weight: bold; color: #949CDF;">
  Project Demo
</h1>

This page presents a demonstration of our waste collection and sorting simulation project.

<h2 style="font-size: 24px; font-weight: bold; color: #949CDF;">
  Overview
</h2>

Our project simulates an arena where two robots collaborate to collect and sort waste:

- `CollectorRobot`: Moves around the arena, detects, and collects waste. It is equipped with a container that can hold up to **`3 waste items`**. Once full, it returns to the initial point to unload the collected waste into a central container.
- `SorterRobot`: Positioned at the unloading point, it waits for waste to be deposited. As soon as the **`first batch`** is delivered, it automatically activates and begins **`sorting waste`** into the appropriate bins based on type.

The simulation uses [Ncurses](Ncurses.md) `** to provide a **` real-time visualization`** of the robots’ movements and actions in the terminal. Every step — collection, transport, unloading, and sorting — is shown live.

## `Demo Video`

<video src="/videos/simulation_demo.webm" controls autoplay muted style="width: 100%; max-width: 800px; height: auto;">
  Your browser does not support the video tag.
</video>

## Key Features Shown in the Demo

- Initialization of the arena, robots, and waste items
- Waste collection by the **`CollectorRobot`**, respecting its container limit
- Real-time visualization using **`Ncurses`**
- Automatic return to base and unloading when the CollectorRobot’s container is full
- **`Automatic activation`** of the **`SorterRobot`** upon the first delivery
- Sorting of waste into the appropriate containers: **`PLASTIC`**, **`METAL`**, **`GLASS`**, and **`ORGANIC`**
- **`Dynamic quantity updates`**: Every sorting action is visually reflected by a decrease in the source container's content and an increase in the destination container corresponding to the waste type
- **`Concurrent execution`** of both robots using **`two separate processes`**, ensuring smooth and coordinated behavior

---

## Environment Overview

The simulation environment is defined in a `map.txt` file and visually represented as a grid made of characters. Each symbol corresponds to a different element in the simulation arena. Here is an example:

```txt
########################################
#                  ##                  #
#  ####    ####     ##     ####    #####
#                                      #
#####  ##########  ######  ##########  #
#                                      #
#  ####  ####  ####  ####  ####  ####  #
#              G                       #
#  ##          P           ##          #
#              M                       #
#                                      #
#              O             P         #
#  ####  ####  ####  ####  ####  ####  #
#                                      #
#  ######  ##########  ######  ######  #
#     G                        M       #
#          ##              ##          #
#  ####    ####    ####    ####    #####
#                O                     #
########################################
```

### Legend

| Symbol | Meaning                                              |
|--------|------------------------------------------------------|
| `#`    | Wall or obstacle — impassable for robots             |
| (space)| Free space — robots can move here                    |
| `P`    | **`Plastic waste`**   |
| `G`    | **`Glass waste`** item                                 |
| `M`    | **`Metal waste`** item                                 |
| `O`    | **`Organic waste`** item                               |

---

## Getting Started

To learn how to compile, run, and test the project, as well as see all required dependencies, check out the [Getting Started Guide](GettingStarted.md).

---

For more technical details, see the [CollectorRobot](CollectorRobot.md) and [SorterRobot](SorterRobot.md) documentation.
