# Ground Control Portal (GCP) — Development Flow & Step-by-Step Roadmap

This document outlines the detailed development phases, sequential launch flows, and pre-flight diagnostics rules for the Ground Control Portal. It serves as the design specification for the implementation phase.

---

## Development Phases

```mermaid
gtpv2
section Development Roadmap
Phase 1: Config Engine     :active, p1, 2026-07-11, 4d
Phase 2: Sequencing Core   : p2, after p1, 4d
Phase 3: Validation Engine : p3, after p2, 5d
Phase 4: Frontend UI       : p4, after p3, 6d
Phase 5: Docker Integration: p5, after p4, 3d
```

### Phase 1: Parameter & Configuration Engine
* **Goal**: Enable the backend to read, parse, and dynamically modify ROS 2 parameter profiles without manual file edits.
* **Steps**:
  1. Write a python-based YAML parser in the FastAPI backend that reads `sim_params.yaml` and `hw_params.yaml`.
  2. Map YAML node parameters (e.g. `square_mission: { ros__parameters: { ... } }`) to structured JSON models.
  3. Implement REST endpoints (`GET /api/missions`, `GET /api/parameters`, `POST /api/parameters/update`).
  4. Write a temp-file generator that outputs a temporary configuration YAML file (e.g. `/tmp/active_run_params.yaml`) with operator adjustments.

### Phase 2: Sequential Process Launcher
* **Goal**: Replace complex, concurrent docker-compose bringups with structured, sequential subprocess spawning.
* **Launch Sequence**:
  1. **Infrastructure**: Check and confirm the Docker daemon is accessible.
  2. **Simulator/Hardware Connection**:
     * If `Simulation`: Start the Gazebo Sim Server (headless or with GUI depending on selection).
     * If `Real`: Verify the serial telemetry/USB link to the Pixhawk flight controller.
  3. **Bridges & Perception**:
     * Start `uxrce_dds_client` on the flight controller.
     * Start the ROS 2 parameters/topics bridges (`ros_gz_bridge`).
     * Launch the camera/IMU bridge and the VIO pipeline (`OpenVINS`).
     * Launch `rosbridge_websocket` and `web_video_server` for visualization.
  4. **Visualization Windows**:
     * Spawn `rqt_image_view` (targeted at `/camera/camera/color/image_raw`).
     * Spawn `rviz2` (with the predefined configuration `quad.rviz`).
  5. **Safety verification**: Pass control to the Validation Engine.

### Phase 3: Pre-Flight Safety Verification Node
* **Goal**: Intercept the mission start if any safety-critical topic, hardware diagnostic, or parameter is out of bounds, alerting the operator instead of allowing a crash.
* **Checks Checklist**:
  * **Parameter Check**: Ensure values like `takeoff_height_m` do not exceed regional limits (e.g. 5.0m for indoor safety nets) and that `max_velocity_m_s` is within performance boundaries.
  * **Telemetry Availability**:
    * If Localization = `GPS`: Verify `/fmu/out/vehicle_gps_position` is active, has a valid RTK/GPS fix, and the satellite count is $>6$.
    * If Localization = `Vision`: Verify `/fmu/in/vehicle_visual_odometry` is publishing continuously ($>10$ Hz). Run a 5-second check to confirm the standard deviation (noise) of the position estimate is below $0.05$ m.
  * **Controller Health**:
    * Query `/fmu/out/battery_status_v1` and verify battery voltage $> 11.5$V (for a 3S battery).
    * Verify the flight controller is reporting `arming_state == STANDBY` (not armed, no fatal pre-flight checks active).
    * Verify `/fmu/out/estimator_status_flags` shows the EKF2 filter is aligned (attitude and position flags are true).
  * **Sequence Logic**:
    * If all checks pass: Return `OK` and launch the selected autonomy node (`square_mission` or `offboard_control_node`).
    * If any check fails: Abort the launch process, terminate child bridges safely, and send a descriptive error alert to the operator console.

### Phase 4: Svelte Frontend & Operator HUD
* **Goal**: Provide a clean, web-based dashboard showing config forms, verification results, console logs, and controls.
* **Views**:
  * **Setup View**: Select Sim/Real, Localization, and Mission. Show the dynamic configuration form.
  * **Verification Terminal**: A real-time diagnostic terminal listing each verification check (e.g. `[CHECK] Battery voltage: 12.1V (PASSED)`, `[CHECK] VIO data flow: 0Hz (FAILED)`).
  * **Active Console Log**: A scrolling viewport displaying the merged stdout/stderr stream from the active launch processes.
  * **Emergency Deck**: Floating widgets containing big, clickable buttons for **EMERGENCY LAND**, **HOLD (Pause Offboard)**, and **DISARM**.

### Phase 5: Container Packaging & Integration
* **Goal**: Build and run the portal container within the existing environment.
* **Steps**:
  1. Write `docker/Dockerfile.gcp` using a lightweight python alpine or slim base image.
  2. Add the `operator-portal` service to `docker-compose.yml`.
  3. Mount `colcon_build_cache` and `ros2_ws` as volumes.
  4. Expose port `8080` (or another free port) to the host.
  5. Add `make portal` to the main `Makefile` to let users start the web portal with a single command.
