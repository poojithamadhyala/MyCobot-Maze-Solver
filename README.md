# 🤖 MyCobot Pro 600 — Autonomous Maze Solver

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![ROS](https://img.shields.io/badge/URDF-Compatible-22314E?style=for-the-badge&logo=ros&logoColor=white)

**A full end-to-end autonomous robotics pipeline: computer vision → path planning → digital twin simulation → physical robot execution.**

*Final Project — Robotics & Autonomous Systems | Arizona State University | December 2024*

</div>

---

## 📌 Overview

This project develops a fully autonomous system for solving 4×4 rectangular mazes using the **MyCobot Pro 600** 6-axis robotic arm. A maze printed on a plastic board is placed within the AI Kit camera's field of view. From there, the system:

1. Captures and processes the maze image using computer vision
2. Detects ArUco markers to localize the workspace in 3D
3. Solves the maze using BFS and extracts turning-point waypoints
4. Converts pixel coordinates to real-world robot coordinates via calibration
5. Plans a smooth trajectory using inverse kinematics in a MATLAB digital twin
6. Transmits joint angle commands to the physical robot via TCP socket programming

> **No human intervention** is required after the maze is placed in the camera zone.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        PERCEPTION LAYER                         │
│   AI Kit Camera → ArUco Detection → Pixel-to-World Calibration  │
│                    (get_coords.py / conversion.py)              │
└────────────────────────────┬────────────────────────────────────┘
                             │ maze_coords.txt
┌────────────────────────────▼────────────────────────────────────┐
│                      PATH PLANNING LAYER                        │
│        BFS Maze Solver → Waypoint Extraction → IK Solver        │
│                         (maze_main.m)                           │
└────────────────────────────┬────────────────────────────────────┘
                             │ angles_maze.csv
┌────────────────────────────▼────────────────────────────────────┐
│                      EXECUTION LAYER                            │
│       Socket Programming → MyCobot Pro 600 Physical Robot       │
│                           (main.py)                             │
└─────────────────────────────────────────────────────────────────┘
```

---

## ✨ Key Features

- **ArUco Marker Localization** — Detects fiducial markers in real-time using OpenCV to establish the robot's workspace frame
- **Pixel-to-World Coordinate Mapping** — Dual linear calibration using 4 reference points for accurate spatial mapping
- **BFS Maze Solving** — Automatically finds the shortest path through any valid maze layout
- **URDF Digital Twin** — Full MATLAB simulation using a custom-built URDF model exported from SolidWorks/Fusion 360
- **Inverse Kinematics** — MATLAB IK solver computes per-waypoint joint configurations with fixed end-effector orientation
- **Trajectory Interpolation** — 15 interpolated points per segment ensure smooth, continuous motion
- **TCP Socket Control** — Real-time command transmission to the physical robot over Ethernet

---

## 📁 Repository Structure

```
mycobot-maze-solver/
│
├── vision/
│   ├── get_coords.py          # ArUco marker detection + real-world coordinate extraction
│   └── conversion.py          # Pixel-to-world coordinate calibration function
│
├── planning/
│   └── maze_main.m            # MATLAB: IK solver, trajectory planning, digital twin animation
│
├── execution/
│   ├── main.py                # Reads angles_maze.csv, streams joint commands to robot via TCP
│   └── single_message.py      # Utility: send a single joint angle command (e.g. home position)
│
├── data/
│   ├── maze_coords1.txt       # 3D waypoints of maze solution path (robot workspace frame)
│   ├── turn_coords.txt        # Extracted turning-point coordinates
│   └── angles_maze.csv        # Joint angles (degrees) generated by MATLAB IK solver
│
├── urdf/
│   └── mycobot_pro600.urdf    # Robot URDF model for MATLAB Robotics Toolbox
│
├── assets/
│   └── maze_1.png             # Input maze image used for path solving
│
└── README.md
```

---

## 🔧 Hardware Requirements

| Component | Details |
|---|---|
| Robot | MyCobot Pro 600 (6-DOF, 600mm reach, ±0.1mm repeatability) |
| Camera | Elephant Robotics AI Kit Camera |
| Computer | Any machine running Python 3.8+ and MATLAB R2023b+ |
| Network | Ethernet connection to robot (IP: configured per setup) |

---

## 💻 Software Dependencies

### Python
```bash
pip install opencv-python numpy sympy
```

### MATLAB
- Robotics System Toolbox
- `importrobot`, `inverseKinematics` functions

---

## 🚀 Getting Started

### Step 1 — Physical Setup
1. Mount the MyCobot Pro 600 on a stable surface
2. Connect the robot to your computer via Ethernet cable
3. On the robot: navigate to **Tools → Configuration → Network/Serial Port** and start the TCP Server
4. Update `SERVER_IP` in `main.py` and `single_message.py` to match your robot's IP address

### Step 2 — Home the Robot
```bash
python execution/single_message.py
```
This sends the robot to its home position:
`[46.484, -120.495, -107.298, -44.824, 88.857, -8.977]`

### Step 3 — Capture Maze & Detect Markers
Place the maze board in the camera zone and run:
```bash
python vision/get_coords.py
```
This detects the ArUco markers, converts pixel positions to robot-frame coordinates, and writes them to `marker_coords.txt`.

### Step 4 — Solve the Maze
Run the maze solving script (Python) to generate `maze_coords1.txt` with the solution path waypoints.

### Step 5 — Plan Trajectory in Digital Twin (MATLAB)
```matlab
run('planning/maze_main.m')
```
This:
- Loads the URDF and initializes the home configuration
- Reads waypoints from `maze_coords1.txt`
- Runs the IK solver for each interpolated point
- Animates the digital twin
- Saves all joint angles to `angles_maze.csv`

### Step 6 — Execute on Physical Robot
```bash
python execution/main.py
```
This reads `angles_maze.csv` and streams each joint configuration to the robot via TCP at 1-second intervals.

---

## 🧮 Coordinate Calibration

The `conversion.py` module maps pixel coordinates `(x_px, y_px)` to robot workspace coordinates `(x_m, y_m)` using a dual linear regression approach with 4 physical reference points:

```python
x_m = mx * x_px + cx
y_m = my * y_px + cy
```

Two independent line fits are averaged for each axis to reduce calibration error. The fixed z-height is held constant at `z = 0.157 m` throughout maze traversal.

---

## 🦾 Inverse Kinematics Setup

The MATLAB IK solver maintains a **fixed end-effector orientation** throughout the trajectory:

```matlab
orientation = eul2quat([0, 0, -90 * pi/180], "XYZ");
```

Joint angle offsets are applied to joints 2 and 4 to align the digital twin's home position with the physical robot. The IK uses the previous solution as its warm-start initial guess, ensuring smooth joint-space transitions between waypoints.

---

## 📊 Results

| Metric | Value |
|---|---|
| Maze Size | 4×4 grid |
| Waypoints | 9 (maze_coords1.txt) |
| Interpolation points per segment | 15 |
| Fixed z-height | 0.157 m |
| Robot speed | 20 (joint speed units) |
| Coordinate accuracy | < ±5mm (empirical) |

---

## 🔮 Future Improvements

- [ ] Replace manual calibration with automatic homography-based transformation using 4+ ArUco markers
- [ ] Add real-time feedback loop using camera to verify end-effector position during execution
- [ ] Support variable z-height for 3D mazes or uneven surfaces
- [ ] Implement ROS2 node wrapper for portability across robot platforms
- [ ] Add collision detection in the digital twin before execution
- [ ] GUI for live visualization of the robot's progress through the maze

---

## 👩‍💻 Author

**Sri Sai Poojitha Madhyala**
M.S. Robotics & Autonomous Systems — Arizona State University
[LinkedIn](https://www.linkedin.com/in/poojithamadhyala-038980323) · [Email](mailto:poojithasrisai795@gmail.com)

---

## 📄 License

This project was developed as part of the RAS graduate program at ASU. Feel free to fork and build on it — just give credit! 🤖

---

## 📚 References

1. Elephant Robotics. *MyCobot Pro 600 Documentation*. https://docs.elephantrobotics.com
2. OpenCV. *ArUco Marker Detection*. https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
3. MathWorks. *Robotics System Toolbox — inverseKinematics*. https://www.mathworks.com/help/robotics/ref/inversekinematics.html
