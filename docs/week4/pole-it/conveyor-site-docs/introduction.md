# `Introduction to Conveyor-Site`

The Conveyor-Site project is a real-time web dashboard for the Waste sorting on the conveyor system. It visualizes the sorting of waste on a conveyor, where each item is detected and classified by color (`GREEN`, `YELLOW`, `RED`, `BLUE`) using an Arduino-based color sensor system.

## `Purpose`

The primary purpose of Conveyor-Site is to provide a user-friendly interface for monitoring, in real time, the number of waste items sorted by color. This helps participants and organizers track sorting efficiency and system performance during the challenge.

## `Dashboard Overview`

[![Vercel Homepage](/week4/images/IT/Website/dashboard_overview.png)](/week4/images/IT/Website/dashboard_overview.png)

[Click here to access the website](https://conveyor-site.vercel.app)

## `How It Works`

- **`Detection`**: An Arduino board with color sensors detects the color of each waste item on the conveyor.
- **`ROS2 Node`**: The detected color is sent from the Arduino to a ROS2 node.
- **`API Communication`**: The ROS2 node forwards the color information to the web dashboard via the `/api/data` endpoint.
- **`Redis Synchronization`**: The backend uses Redis to store and synchronize the color counts in real time.
- **`Real-Time Display`**: The dashboard updates instantly, showing the count of each color as colored cubes.

## `Main Features`

- **`Real-Time Data Visualization`**: The application fetches and displays data in real time, allowing users to see updates as they happen.
- **`Color-Coded Metrics`**: Users can view metrics categorized by colors (RED, GREEN, BLUE, YELLOW), making it easy to identify sorting activity.
- **`API Integration`**: The project includes a robust API for data manipulation and retrieval, ensuring seamless interaction between the ROS2 node, backend, and frontend.
- **`Redis Synchronization`**: Redis is used for fast, reliable, and synchronized storage of color counts across all clients.
  
  [![](/week4/images/IT/Website/redis.png)](/week4/images/IT/Website/redis.png)

- **`Responsive Design`**: The user interface is designed to be responsive, ensuring a smooth experience across various devices and screen sizes.
- **`Reset Functionality`**: Users can reset all counters directly from the interface.

In summary, Conveyor-Site connects hardware (Arduino), middleware (ROS2), Redis, and a modern web dashboard to provide a complete, real-time view of the waste sorting process for the waste sorting system.