# Ackman Webot Workspace

This ROS 2 (Jazzy) project implements a simulated Ackermann-steering robot in Gazebo (Harmonic). The robot is equipped with a camera and uses computer vision to autonomously search for, track, and approach a specific target (a cube) while ignoring decoys like spheres and cylinders.

## Project Structure

The workspace consists of two main ROS 2 packages:

### 1. `webot_description`
This package handles the physical description, simulation environment, and hardware interfaces bridging ROS 2 and Gazebo.
- **`description/`**: Contains the URDF and Xacro files defining the robot's links, joints, camera sensor, and `ros2_control` interfaces.
- **`config/`**: Contains the configuration for `ros2_control` (Ackermann steering and joint state broadcasters) and the `ros_gz_bridge` parameter mappings.
- **`launch/`**: Launch files to spin up `robot_state_publisher`, Gazebo, spawn the robot, and load the controllers (`webot.launch.py`, `rsp.launch.py`).
- **`worlds/`**: Gazebo world file (`shapes.sdf`) containing the ground plane, lighting, a target cube, and various decoy shapes.

### 2. `webot`
This package contains the high-level Python autonomy logic.
- **`perception.py`**: Subscribes to the raw camera feed (`/camera/image_raw`), uses OpenCV to detect a cube using contour analysis and aspect ratios, and publishes the normalized bounding box data to `/perception/box_data`.
- **`control.py`**: A PD controller that subscribes to `/perception/box_data`. It calculates the steering angles and linear velocities required to approach the cube and publishes `TwistStamped` messages to `/ackermann_steering_controller/reference`. It includes search logic if the target is lost.
- **`main.py`**: The main entry point that spins up both the Perception and Control nodes using a MultiThreadedExecutor.

## Dependencies

- ROS 2 (Jazzy)
- Gazebo Sim (Harmonic)
- `ros-jazzy-ros-gz-sim`
- `ros-jazzy-ros-gz-bridge`
- `ros-jazzy-xacro`
- `ros-jazzy-ros2-control`
- `ros-jazzy-gz-ros2-control`
- `ros-jazzy-ackermann-steering-controller`
- `opencv-python`
- `cv_bridge`

## Build Instructions

To build the workspace, use `colcon` from the root of the workspace:

```sh
# Clean previous builds (optional)
rm -rf build/ install/ log/

# Build the workspace
colcon build --symlink-install

# Source the overlay workspace
source install/setup.bash
```

## Running the Simulation
##### 1. Launch the Simulation:
First, launch Gazebo, spawn the robot, start the parameter bridges, and load the controllers:
```sh
ros2 launch webot_description webot.launch.py
```
##### 2. Start the Autonomy Nodes:
In a new terminal, source the workspace and run the main autonomy node:
```sh
source install/setup.bash
ros2 run webot intercept
```


## Controller Logic Details

### Perception
- Filters out contours with area < 500 px  
- Uses `cv2.approxPolyDP` to approximate shapes  
- Selects polygons with **4–6 vertices**  
- Computes **solidity** (contour area / convex hull area) to reject irregular shapes  
- Applies **bounding box aspect ratio constraints**  
- Ensures detection of the **target box**, not decoys (sphere, capsule, cylinder)

---

### Control
- Objective: minimize **X-axis error (turning error)**  
- Uses a **PD Controller**:

\[
u(t) = K_p \cdot e(t) + K_d \cdot \frac{de(t)}{dt}
\]

Where:
- \( e(t) \): lateral error (difference from center)
- \( K_p \): proportional gain  
- \( K_d \): derivative gain  

---

### Additional Logic
- Output is **clamped** to prevent aggressive steering  
- If **no target is detected**:
  - Switches to **rotation-based search mode**