#!/usr/bin/env bash
set -euo pipefail

export HOME="${HOME:-/root}"
export USER="${USER:-root}"
export DISPLAY="${DISPLAY:-:1}"
export RESOLUTION="${RESOLUTION:-1400x900}"

rm -f "/tmp/.X${DISPLAY#:}-lock"
rm -f "/tmp/.X11-unix/X${DISPLAY#:}"
mkdir -p "${HOME}/.vnc"

cat > "${HOME}/.vnc/xstartup" <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
startxfce4 &
EOF
chmod +x "${HOME}/.vnc/xstartup"

vncserver "${DISPLAY}" \
  -geometry "${RESOLUTION}" \
  -depth 24 \
  -SecurityTypes None \
  -localhost no \
  >/tmp/vnc.log 2>&1

websockify --web=/usr/share/novnc/ 6080 localhost:5901 >/tmp/novnc.log 2>&1 &

source /opt/ros/humble/setup.bash
source /workspace/vision_guided_robot_ws/install/setup.bash

if [[ "${AUTO_LAUNCH_DEMO:-1}" == "1" ]]; then
  xfce4-terminal \
    --title="Two-wall target search demo" \
    --command="bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/vision_guided_robot_ws/install/setup.bash; ros2 launch vision_guided_robot demo_search_ball_two_walls.launch.py rviz:=false target_search_start_delay_s:=0.5; exec bash'" &

  sleep 8

  xfce4-terminal \
    --title="Annotated image" \
    --command="bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/vision_guided_robot_ws/install/setup.bash; ros2 run rqt_image_view rqt_image_view /ball/annotated_image; exec bash'" &
fi

cat <<'EOF'

Vision-Guided Robot desktop is running.

Open this URL from your host machine:
  http://localhost:6080/vnc.html?autoconnect=true&resize=scale

EOF

tail -F /tmp/vnc.log /tmp/novnc.log
