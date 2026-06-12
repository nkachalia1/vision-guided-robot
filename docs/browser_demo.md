# Browser-Based Docker Demo

This setup lets reviewers run the project in a browser-hosted Linux desktop without installing ROS 2, Gazebo, or RViz on the host machine.

## Requirements

- Docker Desktop or Docker Engine
- 8 GB RAM minimum, 12 GB recommended
- A modern x86_64 machine

The first build downloads ROS 2, Gazebo, Nav2, SLAM Toolbox, desktop packages, and Python dependencies. It can take a while.

## Run

From the repository root:

```bash
docker compose up --build
```

Open:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

The container starts an XFCE desktop through noVNC. By default it launches:

- `demo_search_ball_two_walls.launch.py`
- `rqt_image_view /ball/annotated_image`

## Manual Launch

To start the desktop without auto-launching the demo:

```bash
AUTO_LAUNCH_DEMO=0 docker compose up --build
```

Then open the browser desktop and run:

```bash
source /opt/ros/humble/setup.bash
source /workspace/vision_guided_robot_ws/install/setup.bash
ros2 launch vision_guided_robot demo_search_ball_two_walls.launch.py rviz:=false
```

## Notes

- Rendering uses software OpenGL for portability.
- Gazebo/RViz performance depends heavily on the host machine.
- This is meant as a portfolio review path, not a production deployment.
