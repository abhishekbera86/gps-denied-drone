# =============================================================================
# Makefile — GPS-Denied Autonomous Drone Stack (v3 — PX4-native ROS 2)
# =============================================================================
# Run from the project root: px4_docker_ws/
#
# PHASE 0/1 WORKFLOW:
#   make build     First time only — builds px4-sim + ros2-autonomy images
#   make sim       Starts PX4 SITL (Gazebo Harmonic, headless) + ROS 2 bridge
#   make shell     Bash inside ros2-autonomy (ros2 topic list, etc.)
#   make stop      Shuts everything down
#
# Targets are added here only as the matching container/package actually
# exists — no aspirational targets pointing at services that aren't built.
# =============================================================================

include .env
export

.PHONY: help build build-ws sim sim-gui flight-test mission stop gz-resync shell shell-px4 logs ps clean clean-all

# Mission flown by `make mission` — currently just square (see common_missions).
MISSION ?= square

# Localization source for `make mission`/`make flight-test` — gps (outdoor)
# or vision (indoor VIO). See resource/phase3-gps-denied-localization-source.md.
LOCALIZATION ?= gps

# When LOCALIZATION=vision, which VIO backend feeds it — loopback (Milestone A
# fake-VIO stand-in, no camera needed) or openvins (Milestone B real VIO).
VIO_BACKEND ?= loopback

# Gazebo world for `make sim`/`make sim-gui` — empty (default, bare ground) or
# vio_test (colored props, needed for VIO_BACKEND=openvins to have anything to
# track). See resource/phase3-gps-denied-localization-source.md.
PX4_GZ_WORLD ?= empty

help:
	@echo ""
	@echo "  ╔══════════════════════════════════════════════╗"
	@echo "  ║   GPS-Denied Drone — Quick Reference         ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  FIRST TIME ONLY                             ║"
	@echo "  ║    make build      Build Docker images        ║"
	@echo "  ║    make build-ws   Build the ROS 2 workspace  ║"
	@echo "  ╠══════════════════════════════════════════════╣"
	@echo "  ║  DAILY WORKFLOW                              ║"
	@echo "  ║    make sim         Start PX4 SITL + ROS 2    ║"
	@echo "  ║      PX4_GZ_WORLD=empty|vio_test (default     ║"
	@echo "  ║        empty; vio_test for VIO_BACKEND=openvins) ║"
	@echo "  ║    make sim-gui     + Gazebo GUI + cam + RViz2 ║"
	@echo "  ║    make flight-test Fly takeoff-hover-land    ║"
	@echo "  ║    make mission     Fly MISSION=square         ║"
	@echo "  ║      LOCALIZATION=gps|vision (default gps)    ║"
	@echo "  ║      VIO_BACKEND=loopback|openvins            ║"
	@echo "  ║    make gz-resync  Drone invisible in GUI? Run ║"
	@echo "  ║                    this (see Makefile comment) ║"
	@echo "  ║    make shell      Shell into ros2-autonomy   ║"
	@echo "  ║    make shell-px4  Shell into px4-sim          ║"
	@echo "  ║    make logs       Tail all container logs    ║"
	@echo "  ║    make ps         Show container status      ║"
	@echo "  ║    make stop       Stop everything            ║"
	@echo "  ╚══════════════════════════════════════════════╝"
	@echo ""

# =============================================================================
# BUILD (one-time, or after Dockerfile/version changes)
# =============================================================================
build:
	DOCKER_BUILDKIT=1 docker compose --profile sim build

# =============================================================================
# SIMULATION
# =============================================================================
sim:
	@echo "==> Starting px4-sim (PX4 v${PX4_VERSION} SITL + Gazebo Harmonic, headless)..."
	@docker compose --profile sim up -d
	@echo ""
	@echo "  ✓ px4-sim and ros2-autonomy containers are up."
	@echo "    Check PX4 boot log:   make logs"
	@echo "    Check ROS 2 topics:   make shell   (then: ros2 topic list)"
	@echo ""

# Same as `make sim` but pops the Gazebo Harmonic GUI on the host desktop so
# you can WATCH the drone fly, PLUS a live rqt_image_view window subscribed
# to /camera/camera/color/image_raw from the start (see docker-compose.gui.
# yml's header comment for why it starts now rather than being launched
# manually per mission — a sequencing race with camera_imu_bridge), PLUS an
# RViz2 window (state_tf_publisher + common_perception/launch/viz.launch.py)
# showing the vehicle's live TF and its actually-flown path (/drone/path) —
# TF/path only, deliberately no camera image in RViz (see viz.launch.py's
# docstring — RViz's Image plugin double-subscribing the same high-
# bandwidth stream as rqt-viewer was visibly laggy, a real bug found and
# fixed 2026-07-10, resource/Vio_Drift_analysis.txt). RViz2's TF/path
# appear immediately (PX4 publishes /fmu/out/vehicle_odometry as soon as
# its estimator initializes, no mission needed); rqt-viewer's image stays
# blank until a VIO mission (localization_source:=vision
# vio_backend:=openvins) is actually flying. The rviz2 container runs
# common_perception (bind-mounted, colcon-built)
# rather than a baked-in apt package, so `make build-ws` must have run at
# least once before `make sim-gui` or that container exits immediately with
# a "Workspace not built yet" message (check: make ps / make logs).
# Layers docker-compose.gui.yml (HEADLESS=0 + X11/DRI passthrough for all
# three windows). Needs an X session (DISPLAY set). If a window is black or
# gz crashes on the GPU, retry with software rendering:
#   GZ_SW_RENDER=1 make sim-gui
sim-gui:
	@echo "==> Starting px4-sim WITH Gazebo GUI (world=${PX4_GZ_WORLD}, DISPLAY=$${DISPLAY:-:0})..."
	@xhost +local:root >/dev/null 2>&1 || echo "  ! xhost not available — GUI may be denied X access"
	@docker compose -f docker-compose.yml -f docker-compose.gui.yml --profile sim up -d
	@echo ""
	@echo "  ✓ Containers up. The Gazebo Harmonic window, an rqt_image_view"
	@echo "    camera preview, and an RViz2 window (TF + path only) should all"
	@echo "    open shortly (rqt_image_view stays blank/black until a VIO"
	@echo "    mission — localization_source:=vision vio_backend:=openvins —"
	@echo "    is actually flying and publishing)."
	@echo "    (First boot takes ~20-40 s. Watch it:   make logs)"
	@echo "    Then, in another terminal:   make flight-test   or   make mission"
	@echo ""

# One-time (or after adding/removing a ROS 2 package, or editing a
# package.xml/setup.py): colcon-builds the whole workspace with
# --symlink-install into the persistent /ros2_ws_build volume. After this,
# editing Python/launch/config files under ros2_ws/src/ takes effect
# immediately on the next `make flight-test`/`make mission` — NO rebuild —
# because symlink-install symlinks source into the install tree rather than
# copying it. Only re-run this when the package *set* or *installed files
# list* changes (setup.py's data_files, a new node, etc.), not for tuning
# sim_params.yaml or editing a mission's Python.
build-ws:
	@echo "==> Building the ROS 2 workspace (common_control, common_missions, common_perception, sim_bringup, hw_bringup)..."
	@docker exec -it ros2-autonomy bash -c "\
		source /opt/ros/humble/setup.bash && \
		source /opt/px4_ros2_ws/install/setup.bash && \
		mkdir -p /ros2_ws_build && cd /ros2_ws_build && \
		colcon build --symlink-install --base-paths /ros2_ws/src \
			--build-base /ros2_ws_build/build \
			--install-base /ros2_ws_build/install"
	@echo "  ✓ Workspace built. make flight-test / make mission won't rebuild again"
	@echo "    unless you add a package or change a setup.py/package.xml."

# Fly the Phase 1 test through the sim_bringup layer: arm → offboard →
# takeoff to 2 m → hover → land → disarm. PX4's EKF2 needs ~30-60 s after
# `make sim` before preflight checks pass — the node retries once per second
# until PX4 accepts, so just leave it running. Params come from
# sim_bringup/config/sim_params.yaml — see §7 in README.md to retune; no
# rebuild needed for that, just re-run this target.
flight-test:
	@docker exec -it ros2-autonomy bash -c "\
		test -f /ros2_ws_build/install/setup.bash || { echo 'Workspace not built yet — run: make build-ws'; exit 1; }; \
		echo '==> Running the offboard flight test (via sim_bringup)...'; \
		source /opt/ros/humble/setup.bash && \
		source /opt/px4_ros2_ws/install/setup.bash && \
		source /opt/openvins_ws/install/setup.bash && \
		source /ros2_ws_build/install/setup.bash && \
		ros2 launch sim_bringup sim.launch.py action:=hover \
			localization_source:=$(LOCALIZATION) vio_backend:=$(VIO_BACKEND)"

# Fly a named waypoint mission (Phase 2): make mission MISSION=square.
# Through the sim_bringup entry point; the mission takes off, flies its
# waypoint sequence, returns and lands. To retune geometry (side_length_m,
# area_length_m, ...), edit sim_bringup/config/sim_params.yaml and re-run
# this target directly — no `make build-ws` needed for a config-only change.
mission:
	@docker exec -it ros2-autonomy bash -c "\
		test -f /ros2_ws_build/install/setup.bash || { echo 'Workspace not built yet — run: make build-ws'; exit 1; }; \
		echo \"==> Flying the '$(MISSION)' mission, localization=$(LOCALIZATION) (via sim_bringup)...\"; \
		source /opt/ros/humble/setup.bash && \
		source /opt/px4_ros2_ws/install/setup.bash && \
		source /opt/openvins_ws/install/setup.bash && \
		source /ros2_ws_build/install/setup.bash && \
		ros2 launch sim_bringup sim.launch.py action:=mission mission:=$(MISSION) \
			localization_source:=$(LOCALIZATION) vio_backend:=$(VIO_BACKEND)"

stop:
	@docker compose -f docker-compose.yml -f docker-compose.gui.yml --profile sim down --remove-orphans
	@echo "  ✓ All containers stopped."

# =============================================================================
# DEBUGGING
# =============================================================================
# Fix for "the drone doesn't appear in the Gazebo GUI window" — the
# world/props render, but the vehicle (spawned dynamically by PX4 AFTER the
# world/GUI are up, not defined in the world file itself) doesn't, and it's
# missing from the GUI's entity tree too even though the sim is flying it
# correctly (verify with: gz service -s /world/<w>/scene/info — the vehicle
# is fully there server-side). Cause: under heavy boot load (low real-time
# factor), the GUI client's initial scene sync can time out AND miss the
# later entity-spawn notification; once desynced it never catches up on its
# own. A `gz service .../scene/info` call does NOT fix it — the response
# goes to the service CALLER, not the GUI (tried; confirmed ineffective).
# What works: restart just the GUI client — it reconnects and downloads the
# complete scene fresh. The sim server, PX4, and any in-progress flight are
# completely unaffected (the GUI is a pure viewer). The relaunch must
# re-export GZ_SIM_RESOURCE_PATH (mesh model:// resolution) and GZ_IP —
# those live in PX4's launch-script session, not the container's top-level
# env that `docker exec` inherits.
gz-resync:
	@echo "==> Restarting the Gazebo GUI client (sim server keeps running — flight unaffected)..."
	-@docker exec px4-sim pkill -f "gz sim -g" 2>/dev/null || true
	@sleep 2
	@docker exec -d px4-sim bash -c "export GZ_SIM_RESOURCE_PATH=/PX4-Autopilot/Tools/simulation/gz/models:/PX4-Autopilot/Tools/simulation/gz/worlds GZ_IP=127.0.0.1 QT_X11_NO_MITSHM=1; gz sim -g > /tmp/gz_gui_relaunch.log 2>&1"
	@echo "  ✓ GUI relaunched — a fresh window opens in ~5-20 s with the full scene, drone included."

shell:
	docker exec -it ros2-autonomy bash

shell-px4:
	docker exec -it px4-sim bash

logs:
	docker compose logs -f

ps:
	docker compose ps

# =============================================================================
# MAINTENANCE
# =============================================================================
clean:
	@docker compose --profile sim down --rmi local 2>/dev/null || true
	@echo "  ✓ Local images removed."

clean-all:
	@echo "WARNING: removes all images and the colcon build cache volume."
	@docker compose --profile sim down --rmi all --volumes --remove-orphans 2>/dev/null || true
