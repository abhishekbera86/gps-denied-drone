# Ground Control Portal (GCP) — Conceptual Architecture

This document defines the architecture and design specifications for the **Ground Control Portal (GCP)**. The GCP acts as the primary orchestrator, parameter editor, and pre-flight diagnostics engine for both simulated and real GPS-denied quadcopter flights.

---

## 1. System Scope & Constraints

* **Strict Sandboxing**: The GCP is developed exclusively inside the `/operator_portal` directory. It must never modify code or configurations inside existing packages (`common_control`, `common_missions`, `common_perception`, `sim_bringup`, `hw_bringup`).
* **Visualizer Tooling**:
  * **Camera Streams**: Visualized using `rqt_image_view` on the host/GUI container.
  * **3D Trajectory & Transforms**: Visualized using `rviz2` on the host/GUI container.
  * **Dashboard Console**: The GCP web interface focuses on parameter configuration, sequential launch orchestration, system log consolidation, and pre-flight safety verification.

---

## 2. Core Functional Requirements

### A. Pre-Flight Verification & Safety Engine
Before a mission can be armed or started, the GCP runs a sequential validation pipeline to ensure the system is in a healthy state. If any check fails, the GCP **aborts the launch sequence** and presents an alert to the operator.
* **Parameter Validation**: Checks that all values entered by the user (takeoff heights, velocity limits, etc.) fall within safe predefined bounds.
* **DDS Node Discovery**: Verifies that required ROS 2 nodes (e.g., PX4's uXRCE agent, the bridge nodes) are active on the graph.
* **Topic/Data Flow Validation**:
  * For GPS missions: Verifies that `/fmu/out/vehicle_gps_position` is publishing valid lock coordinates.
  * For Vision/VIO missions: Verifies that `/fmu/in/vehicle_visual_odometry` is active and publishing at the expected rate (e.g., >10 Hz), and that OpenVINS has completed static initialization.
* **Hardware/SITL Status Check**: Confirms communication with the flight controller (battery voltage > threshold, arming state is `STANDBY`, sensor health checks pass).

### B. Dynamic Configuration & Parameter Matching
* **YAML Extraction**: The GCP backend parses parameter keys and default values from `sim_params.yaml` (for simulation) or `hw_params.yaml` (for hardware) on startup.
* **UI Code Generation**: The frontend renders form fields mapping to these parameters, allowing real-time tuning before launch.
* **Run Profile Generation**: Upon submit, the GCP writes these overrides to a temporary run-config YAML file to be passed directly to the launch processes.

### C. Sequential Process Orchestration
Instead of launching everything simultaneously (which can cause startup races under high CPU loads), the GCP initiates services sequentially:
1. **Simulation Engine / Hardware Bridge**: Starts Gazebo (or initiates telemetry link for hardware).
2. **Perception Bridge & VIO**: Starts `ros_gz_bridge`, camera/IMU bridges, and OpenVINS.
3. **Visualization Windows**: Spawns `rqt_image_view` and `rviz2` in the GUI context.
4. **Pre-Flight Diagnostics Node**: Runs the telemetry verification script.
5. **Flight Autonomy Node**: Launches the selected mission (e.g., `square` or `hover`) once all diagnostics report `OK`.

---

## 3. High-Level Architecture

The portal backend acts as a central supervisor. It parses configurations, exposes REST/WebSocket endpoints, manages child subprocesses, and aggregates system topics.

```
+-------------------------------------------------------------+
|                     OPERATOR PORTAL                         |
|                                                             |
|  +-------------------+              +--------------------+  |
|  |   Web Frontend    | <=========>  |    FastAPI Server  |  |
|  |  (Config & Logs)  |  WebSockets  | (Process Supervisor)|  |
|  +-------------------+              +----------+---------+  |
+------------------------------------------------|------------+
                                                 |
                   +-----------------------------+-----------------------------+
                   | Subprocess Execution        | Configuration Output        | Telemetry Ingestion
                   v                             v                             v
        +---------------------+       +---------------------+       +----------------------+
        |  ROS 2 Launch / CLI |       | Temporary YAML File |       |  rosbridge_websocket |
        +----------+----------+       +----------+----------+       +----------+-----------+
                   |                             |                             |
                   |                             |                             |
                   v                             v                             v
    +--------------------------------------------------------------------------------------+
    |                                   ROS 2 GRAPH                                        |
    |                                                                                      |
    |  +------------------+     +--------------------+     +-------------+     +--------+  |
    |  | Autonomy Mission |     |  OpenVINS Pipeline |     |   Gazebo    |     | RViz2  |  |
    |  +------------------+     +--------------------+     +-------------+     +--------+  |
    +--------------------------------------------------------------------------------------+
```

---

## 4. Technology Stack & Docker Design

* **Container Isolation**: A separate Docker image/container `operator-portal` will be added to the Docker Compose network.
* **Shared Workspace Volumes**: The portal container will mount `colcon_build_cache` and the source folder `ros2_ws` as read-only volumes to inspect parameter files and source directories.
* **Backend Stack**:
  * **Language**: Python 3 (matching ROS 2 environment).
  * **Framework**: FastAPI (for high-performance async endpoints and native WebSockets support).
  * **Process Management**: Python `asyncio.create_subprocess_exec` to spawn and track ROS 2 launch sequences.
* **Frontend Stack**:
  * **Framework**: Svelte (minimal footprint, compiles down to vanilla JS, runs cleanly inside containers).
  * **Communication**: `roslibjs` for direct telemetry connection, and standard HTTP requests to the FastAPI backend.
