# Robotic Arm Pick-and-Place Automation (6-DOF)

This repository details a sophisticated simulation of a 6-DOF robotic arm performing an automated pick-and-place task. The project leverages **ROS2 Foxy/Humble**, **Gazebo**, and **MoveIt2** to handle complex dynamics, motion planning, and collision avoidance.

---

## Project Steps and Key Technologies

### 1. Modeling the Robot (URDF/Xacro)
* **Goal:** Define the robot's kinematics, dynamics, and visual properties.
* **Technologies:** Gazebo, URDF (Unified Robot Description Format), Xacro.
* **Files:** `robot_description/urdf/arm.urdf.xacro`

### 2. Dynamics and Simulation
* **Goal:** Create a physics-accurate environment for testing controllers and motion.
* **Technologies:** Gazebo, `ros2_control` (simulated hardware interface).

### 3. Motion Planning Integration (MoveIt2)
* **Goal:** Integrate the kinematics solver, define planning groups, and configure collision avoidance.
* **Technologies:** MoveIt2 Setup Assistant, RViz2.

### 4. Implementing the Pick-and-Place Logic
* **Goal:** Develop the high-level ROS2 node to sequence motion plans, gripper actions, and task logic.
* **Technologies:** ROS2 Python (`rclpy`), MoveIt2 Planning Interface.
* **Files:** `robot_control/robot_control/pick_and_place_node.py`

---

## Getting Started

1.  **Clone** this repository into your ROS2 workspace `~/ros2_ws/src/`.
2.  **Install dependencies** (e.g., `ros-humble-moveit`, `ros-humble-gazebo-ros-pkgs`, `ros-humble-xacro`).
3.  **Create MoveIt2 Configuration:** Run the MoveIt Setup Assistant on the `arm.urdf.xacro` file to create your `arm_moveit_config` package (or replace the placeholder name).
4.  **Build:** `colcon build`.
5.  **Launch:** `source install/setup.bash && ros2 launch robot_control pick_and_place_launch.py`
