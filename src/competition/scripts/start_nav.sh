#!/bin/bash
# Wrapper to start navigation bringup AND rviz with proper environment sourced.
# This script is invoked by the competition node via the "nav_launch_cmd" parameter.
# It will start both navigation bringup and rviz as child processes, forward
# SIGINT/SIGTERM to them, and wait until they exit. Keep this script running
# while the launched components are alive so the caller (competition node)
# can keep track of the process.

# --- configuration (edit if needed) ---
: ${ROS_DISTRO:=humble}
WORKSPACE_INSTALL_DIR="/home/ubuntu/ros2_ws/install"
MAP_PATH="/home/ubuntu/ros2_ws/src/slam/maps/map_03.yaml"
PARAMS_PATH="/home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml"
RVIZ_CONFIG="/home/ubuntu/ros2_ws/src/navigation/rviz/navigation_transport.rviz"

# --- source ROS & workspace if available ---
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -f "$WORKSPACE_INSTALL_DIR/setup.bash" ]; then
  source "$WORKSPACE_INSTALL_DIR/setup.bash"
fi

# child pids
NAV_PID=0
RVIZ_PID=0

term_handler() {
  # forward termination to children
  [ "$NAV_PID" -ne 0 ] && kill -TERM "$NAV_PID" 2>/dev/null || true
  [ "$RVIZ_PID" -ne 0 ] && kill -TERM "$RVIZ_PID" 2>/dev/null || true
}

trap term_handler SIGTERM SIGINT

# start navigation bringup in background
ros2 launch navigation bringup.launch.py \
  use_sim_time:=false \
  map:="$MAP_PATH" \
  params_file:="$PARAMS_PATH" \
  namespace:="/" \
  use_namespace:=false \
  autostart:=true \
  use_teb:=true &
NAV_PID=$!
echo "started navigation bringup (pid=$NAV_PID)"

# small delay to let nav nodes bring up
sleep 0.5

# start rviz in background
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!
echo "started rviz (pid=$RVIZ_PID)"

# wait for either process to exit; when one exits, forward termination to the other
while true; do
  if ! kill -0 "$NAV_PID" 2>/dev/null; then
    echo "navigation process exited"
    [ "$RVIZ_PID" -ne 0 ] && kill -TERM "$RVIZ_PID" 2>/dev/null || true
    break
  fi
  if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "rviz process exited"
    [ "$NAV_PID" -ne 0 ] && kill -TERM "$NAV_PID" 2>/dev/null || true
    break
  fi
  sleep 0.5
done

# wait for children to actually exit
[ "$NAV_PID" -ne 0 ] && wait "$NAV_PID" 2>/dev/null || true
[ "$RVIZ_PID" -ne 0 ] && wait "$RVIZ_PID" 2>/dev/null || true

echo "start_nav.sh exiting"
exit 0
