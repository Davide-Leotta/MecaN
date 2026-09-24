# Mecanum Wheeled Robot Navigation Simulator

This repository contains the project for an omnidirectional robot (equipped with Mecanum wheels) simulated in **Godot Engine**. The robot navigates autonomously through an unknown map populated by both static and dynamic obstacles. The robot's "brain" (control system and pathfinding) is developed entirely in **Python** and analyzed using **Jupyter Notebook**, communicating in real-time with the game engine.

## Key Features

- **Omnidirectional Kinematics:** Accurate physics and control of a Mecanum-wheeled robot in Godot, capable of translating and rotating simultaneously in any direction.
- **Real-Time LIDAR Sensor:** Environment detection via a simulated LIDAR (raycasting) that allows the robot to "see" both map boundaries and moving objects.
- **Blind Navigation:** The robot has no pre-loaded map (unknown environment). It makes decisions based exclusively on local odometry and instantaneous LIDAR data.
- **Decoupled Control:** The decision-making logic resides in Python scripts and `.ipynb` files, ensuring flexibility for data analysis and algorithmic testing without the need to recompile the game.

## Implemented Pathfinding Algorithms

The Python control system includes the implementation of two reactive approaches for navigation and obstacle avoidance:

1. **Artificial Potential Field (APF):** 
   The robot is subject to virtual forces. The target generates an *attractive* force, while obstacles detected by the LIDAR generate *repulsive* forces. The vector sum of these forces determines the instantaneous movement direction (leveraging the extreme mobility of the Mecanum wheels).
2. **Bug-0 Algorithm:**
   A purely reactive approach where the robot moves in a straight line toward the goal. When the LIDAR detects an obstacle in its path, the robot follows the obstacle's perimeter until the path toward the target is clear again.

---

## Technologies Used

*   **Simulation:** Godot Engine 4.7.2
*   **Control Language:** Python 3.14.7
*   **Data Analysis & Prototyping:** Jupyter Notebook (`.ipynb`)
*   **Communication:** Custom lightweight DDS (Data Distribution Service) over **UDP Sockets**.
*   **Python Libraries:** `numpy`, `opencv-python` (`cv2`), `matplotlib`

---

## Repository Structure

```text
├── godot/                 # Godot project containing scenes, the robot, and GDScript files
│   ├── scripts/           # Scripts for robot physics and LIDAR data acquisition
│   └── ...
├── python/                # Python control system
│   ├── lib/               # Implementation of Potential Field and Bug-0
│   │   ├── bug.py         # Bug-0 algorithm implementation
│   │   ├── dataplot.py    # Data plotting and visualization tools
│   │   ├── dds.py         # Custom UDP Publish/Subscribe protocol
│   │   ├── pot_field.py   # Artificial potential Field implementation
│   │   ├── system.py      # Core system functions
│   │   └── time.py        # Timing utilities
│   ├── control.ipynb      # Jupyter Notebook entry point for connecting to Godot
│   └── control.py         # Python script entry point for connecting to Godot
├── .gitignore
└── README.md
