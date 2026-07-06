# GPS-Denied Autonomous Drone Stack

> PX4 v1.14.3 · ROS 2 Humble · Aerostack2 · OpenVINS VIO · Fully Containerised

This repository contains a professional-grade, containerised autonomous drone stack built on the **Aerostack2 (AS2)** framework. It enables a quadcopter to fly **without GPS** in both simulation and physical real-world environments using Visual-Inertial Odometry (VIO) from an Intel RealSense D435i depth camera processed by OpenVINS.

The key design pattern is **Sim-to-Real parity**: the exact same high-level Python mission logic runs in simulation and on real companion computer hardware (Orange Pi 5 + Pixhawk flight controller).

---

## 1. Prerequisites & Environment Setup

This stack runs completely inside Docker to avoid any host dependency contamination, library compilation, or system-level setup.

### Host Machine Requirements
* **Operating System**: Ubuntu 22.04 LTS (or any Linux distribution running Docker).
* **Architecture**: x86_64 for Development/Simulation; ARM64 (e.g., Orange Pi 5) for Real Flight hardware.
* **Hardware**: Dual-core/Quad-core CPU with at least 8 GB RAM.
* **Graphics**: **No GPU/graphics acceleration is required** for simulation. The stack runs completely headless.

### Host Package Installation
To install Docker Engine and Docker Compose, run the following commands on your host:

```bash
# 1. Download and run the official Docker installation script
curl -fsSL https://get.docker.com | bash

# 2. Add your current user to the docker group so you don't need sudo for every command
sudo usermod -aG docker $USER

# 3. Apply the group changes to the current terminal session
newgrp docker

# 4. Verify Docker and Docker Compose are installed correctly
docker --version
docker compose version
```

---

## 2. Quick Start: Simulation Flight

### Step 1 — Clone the Repository and Navigate to the Workspace
Clone this repository to your development directory:

```bash
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone
```

### Step 2 — Build the Docker Image
Build the custom Aerostack2 image, which installs extra dependencies (such as Cyclone DDS and MAVLink helper modules) on top of the pre-built base image. This step takes 2–3 minutes:

```bash
make build
```

### Step 3 — Launch the Simulation World
Launch the simulation containers and the Aerostack2 node stack:

```bash
make sim
```

#### Under the Hood of `make sim`:
1. Starts the `aerostack2` container in host networking mode.
2. Triggers `scripts/launch_sim.sh` inside the container.
3. Automatically runs `colcon build` inside the container to register the `quad_core` and `quad_sim` ROS 2 packages.
4. Calls `ros2 launch quad_sim sim.launch.py` to spin up the autonomy nodes:
   * **`as2_platform_multirotor_simulator`**: The built-in simulator node simulating drone physics.
   * **`as2_state_estimator`**: Ingests ground truth simulation state.
   * **`as2_motion_controller`**: Receives control setpoints and generates flight actuator inputs.
   * **`as2_behaviors_motion`**: High-level action servers (Takeoff, Land, GoTo).
5. **Keeps the terminal open** to output live logs from all running nodes.

*Leave this terminal running and open a **new terminal session**.*

### Step 4 — Run the Autonomous Mission
In the new terminal session, execute the pre-built mission:

```bash
# Ensure you are in the cloned repository directory
cd gps-denied-drone

# Launch the Python-based autonomous mission script
make mission
```

#### Expected Simulation Behavior:
1. The script connects to the drone interface (`drone0`).
2. Drone **arms** and switches to **Offboard mode**.
3. **Takes off** to 2.0 metres.
4. Flies a **4-point square pattern** (4.0m x 4.0m) maintaining a height of 2.0m.
5. Returns to the takeoff origin (0, 0).
6. **Lands** safely and **disarms**.

### Step 5 — Stop and Clean Up
To stop the simulation nodes and clean up the container resources, run:

```bash
make stop
```

---

## 3. ROS 2 Nodes & Topics Architecture

Once the simulation stack is running (`make sim`), you can query ROS 2 nodes and topics by opening a shell in the container:

```bash
make shell
```

Inside the container shell, run standard ROS 2 commands:

### Running Nodes
* **`/drone0/platform`**: Node managing simulation dynamics or serial MAVLink connection to the Pixhawk.
* **`/drone0/state_estimator`**: Feeds current state estimation to the controller.
* **`/drone0/motion_controller`**: Implements differential flatness algorithms to translate high-level trajectories to platform attitude/thrust setpoints.
* **`/drone0/behaviors/`**: Action servers managing discrete movements (e.g., `/drone0/behaviors/takeoff`, `/drone0/behaviors/land`, `/drone0/behaviors/go_to`).

### Key ROS 2 Topics
* **`/drone0/self_localization/pose`** [`geometry_msgs/msg/PoseStamped`]: Current estimated position (ENU coordinates).
* **`/drone0/self_localization/twist`** [`geometry_msgs/msg/TwistStamped`]: Current estimated linear and angular velocities.
* **`/drone0/actuator_command/twist`** [`geometry_msgs/msg/TwistStamped`]: Output velocity command from the controller.
* **`/drone0/sensor_measurements/imu`** [`sensor_msgs/msg/Imu`]: IMU measurements coming from the simulation or the D435i camera.
* **`/openvins/odometry`** [`nav_msgs/msg/Odometry`]: Output odometry from OpenVINS VIO (in VIO/hardware mode).

To echo a topic in real-time inside the container:
```bash
ros2 topic echo /drone0/self_localization/pose
```

---

## 4. Writing & Running Custom Missions

Autonomous missions are written using the **Aerostack2 Python API**. All autonomy logic lives inside `ros2_ws/src/quad_core/mission.py`. 

### Python API Structure
Here is a simplified blueprint of how to build a custom flight script:

```python
import rclpy
import sys
from as2_python_api.drone_interface import DroneInterface

def main():
    rclpy.init()

    # 1. Initialize drone interface matching the namespace (e.g., drone0)
    drone = DroneInterface(drone_id="drone0", verbose=False)

    try:
        # 2. Arm and switch to Offboard
        print("Arming...")
        if drone.arm():
            
            # 3. Takeoff behavior
            print("Taking off...")
            drone.takeoff(height=1.5, speed=0.5)
            
            # 4. GoTo waypoint behavior
            print("Moving to waypoint...")
            drone.go_to.go_to_point_with_yaw(x=2.0, y=0.0, z=1.5, angle=0.0, speed=0.5)
            
            # 5. Land behavior
            print("Landing...")
            drone.land(speed=0.4)
            
            # 6. Disarm
            drone.disarm()
            
    except KeyboardInterrupt:
        print("Interrupted! Attempting emergency landing...")
        drone.land(speed=0.4)
    finally:
        drone.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### How to Create and Run a Custom Mission:
1. Create a new Python file on your host machine inside `ros2_ws/src/quad_core/` (e.g., `ros2_ws/src/quad_core/my_custom_mission.py`).
2. Implement your flight pattern using the `DroneInterface` methods.
3. Edit the `Makefile` or run the script directly inside the container:
   ```bash
   # Run your custom mission using make command structure:
   docker exec -it aerostack2 python3 /ros2_ws/src/quad_core/my_custom_mission.py
   ```
   *Note: Because the directory `./ros2_ws` is live-mounted from your host to the docker container, you do not need to rebuild the Docker image to update Python mission code. Simply save the file on your host and run it inside the container.*

---

## 5. Sim-to-Real Hardware Deployment

To transition your flight stack to real hardware, use an Orange Pi 5 companion computer connected to a Pixhawk autopilot via a UART serial link.

### Hardware Architecture:
```
Intel RealSense D435i  ──(USB 3.0)──>  Orange Pi 5  ──(TELEM2 UART)──>  Pixhawk (PX4)
```

### Step-by-Step Hardware Execution:
1. Connect the RealSense D435i to a USB 3.0 port on the Orange Pi.
2. Connect TELEM2 TX/RX pins from the Pixhawk to the RX/TX UART pins on the Orange Pi.
3. Flash the Pixhawk parameters to disable GPS and enable external vision height/odometry input:
   * Load the `ros2_ws/src/quad_core/config/ekf2_vio.params` file via QGroundControl.
   * Reboot the Pixhawk.
4. Clone the repository on the Orange Pi and build the hardware image:
   ```bash
   git clone https://github.com/abhishekbera86/gps-denied-drone.git
   cd gps-denied-drone
   
   # Build the ARM64 image with librealsense SDK v2.58.2 + OpenVINS VIO
   make build-hw
   ```
5. Launch the hardware stack on the Orange Pi:
   ```bash
   make hw
   ```
   This compiles your local workspace and launches `ros2 launch quad_real hw.launch.py`, starting the camera drivers, the Pixhawk serial bridge, the state estimator, and the motion controller.
6. Execute the identical mission file:
   ```bash
   make mission
   ```

---

## 6. Commands Reference Cheat Sheet

| Command | Action | Location |
|---|---|---|
| `make build` | Builds the default Aerostack2 image | Dev Host |
| `make sim` | Starts container and launches the simulation nodes | Dev Host |
| `make mission` | Executes the high-level Python mission script | Dev Host / OPi 5 |
| `make stop` | Shuts down and cleans up all running containers | Dev Host / OPi 5 |
| `make shell` | Enters an interactive bash terminal inside the running container | Dev Host / OPi 5 |
| `make logs` | Tails live logging from all active container services | Dev Host / OPi 5 |
| `make health` | Verifies ROS 2 topics and node health status | Dev Host / OPi 5 |
| `make build-hw` | Builds the hardware-specific container image (ARM64) | Orange Pi 5 |
| `make hw` | Launches the real hardware drone nodes (RealSense + Pixhawk UART) | Orange Pi 5 |
| `make ps` | Displays currently active Docker containers | Dev Host / OPi 5 |
