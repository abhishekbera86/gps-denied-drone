# Operator Portal

This package contains the **Ground Control Portal (GCP)** for the GPS-denied autonomous drone stack. It is an independent utility package decoupled from the core flight control, estimation, and simulation components.

---

## 1. System Architecture

The following diagram illustrates the lifecycle coordination and network topologies. The Operator Portal acts as an external supervisor, interacting with the ROS 2 graph and the Docker Compose setup.

```mermaid
graph TB
    subgraph Host / Operator Machine
        Browser[Operator Web UI]
        RViz[rviz2 (TF & Paths)]
        Rqt[rqt_image_view (Camera Feed)]
    end

    subgraph operator-portal Container
        Backend[FastAPI Supervisor Node]
        ParamParser[YAML Parameter Engine]
        SubProc[Process Launch Controller]
    end

    subgraph ros2-autonomy Container
        RosBridge[rosbridge_server]
        WebVideo[web_video_server]
        MissionNode[Autonomy Mission Node]
        VioNode[OpenVINS Estimator]
    end

    subgraph px4-sim Container
        PX4[PX4 Autopilot SITL]
        Gz[Gazebo Sim Server]
    end

    %% Configuration & Launch Actions
    Browser -- "1. Configure Params" --> Backend
    Backend -- "2. Parse / Write" --> ParamParser
    Browser -- "3. Start Mission" --> Backend
    Backend -- "4. Spawn Launch Process" --> SubProc
    SubProc -- "docker exec / SSH" --> MissionNode

    %% Pre-Flight Validation & Telemetry
    Browser -- "WebSockets (Telemetry & Health)" --> RosBridge
    RosBridge -- "Subscribes to Topics" --> MissionNode
    MissionNode -- "MAVLink / DDS" --> PX4

    %% Visualizer Subsystems
    Rqt -- "Subscribes raw image" --> Gz
    RViz -- "Subscribes /drone/path & TFs" --> MissionNode
```

---

## 2. Directory Structure

```
operator_portal/
├── README.md                 # System architecture overview (this file)
├── development_flow.md       # Step-by-step developer roadmap
└── resources/
    └── ground_control_portal_plan.md  # Detailed design document
```

---

## 3. Scope of Operation

All code and templates for the portal must remain strictly within the `operator_portal/` directory. 
* **Do not modify** files in `/ros2_ws/src/common_control`, `/ros2_ws/src/common_missions`, `/ros2_ws/src/common_perception`, `/ros2_ws/src/sim_bringup`, or `/ros2_ws/src/hw_bringup`.
* Communication between the portal and the flight stack is restricted to **ROS 2 topics, actions, services**, or via generating parameter override files at runtime.
