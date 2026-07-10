"""viz.launch.py — live visualization: state_tf_publisher + RViz2.

Shows the vehicle's current pose (TF, `odom` -> `base_link`) and the path
it has actually flown (`/drone/path`) — 3D spatial data only. Deliberately
does NOT show the camera image: RViz2's Image display and rqt_image_view
both subscribing to the same high-bandwidth raw stream
(`/camera/camera/color/image_raw`, ~83 MB/s at 1280x720@30Hz) doubled DDS/
CPU load for no benefit, and RViz's Image plugin uploads a fresh GPU
texture on the same render thread as the 3D view, competing with it —
confirmed live as visibly laggy/stale compared to `rqt-viewer`'s dedicated
2D widget (resource/Vio_Drift_analysis.txt). Use `rqt-viewer`
(docker-compose.gui.yml) for the camera feed; this window is TF/path only.

Sim/hw-agnostic on purpose: state_tf_publisher only touches topics PX4
publishes identically in either environment (/fmu/out/vehicle_odometry).
docker-compose.gui.yml's `rviz2` service runs this for sim; a hardware GUI
overlay can include the exact same launch file unchanged in Phase 4.

Deliberately separate from common_missions/autonomy.launch.py — started
once, up from the start of the sim/hw session, so RViz keeps showing the
vehicle across multiple mission runs rather than tearing down with each
mission's own on_exit=Shutdown(). See state_tf_publisher.py's docstring.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('common_perception'), 'config', 'quad.rviz')

    return LaunchDescription([
        Node(
            package='common_perception',
            executable='state_tf_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
