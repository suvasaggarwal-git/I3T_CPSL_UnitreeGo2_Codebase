#!/bin/bash
# Start the 3D SLAM (no Nav2) in one Terminator window on this dog, with RViz:
#
#   driver (dog.launch.py)    | sensors (Livox)
#   3D SLAM (RTAB-Map), +20 s | RViz: 3D cloud map, 2D map, LiDAR, YOLO image and objects, +25 s
#
# Run it on the dog's desktop (e.g. through NoMachine): alias slam3d. Ctrl-C in a pane
# stops that part; closing the window stops everything.

WS="$HOME/I3T_CPSL_UnitreeGo2_codebase"
RVIZ="$WS/install/cpsl_nav/share/cpsl_nav/rviz_cfgs/slam_3d_yolo.rviz"
CFG="${XDG_RUNTIME_DIR:-/tmp}/slam3d.terminator"

if [ -z "$DISPLAY" ]; then
    echo "No display: run this on the dog's desktop (e.g. through NoMachine)." >&2
    exit 1
fi
RUNNING="bin/ros2 launch|lib/rviz2/rviz2"
if pgrep -f "$RUNNING" >/dev/null; then
    echo "ROS is already running on this dog:" >&2
    pgrep -af "$RUNNING" | cut -c1-120 >&2
    echo "Close its window first, or: pkill -INT -f '$RUNNING'" >&2
    exit 1
fi

# The panes inherit this environment (as ~/.bashrc sets it up).
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/unitree_msgs_ws/install/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml

# A pane's command: wait $1 s, run $2, then keep the pane open with a shell.
# (No commas or quotes: they would break Terminator's config syntax.)
pane() {
    echo "sleep $1; $2; echo; echo [$3 ended - Ctrl-D closes this pane]; exec bash"
}

cat > "$CFG" <<EOF
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    scrollback_lines = 10000
[layouts]
  [[slam3d]]
    [[[window0]]]
      type = Window
      parent = ""
      title = Go2 3D SLAM
      size = 1500, 900
    [[[root]]]
      type = VPaned
      parent = window0
      order = 0
    [[[top]]]
      type = HPaned
      parent = root
      order = 0
    [[[bottom]]]
      type = HPaned
      parent = root
      order = 1
    [[[driver]]]
      type = Terminal
      parent = top
      order = 0
      command = $(pane 0 "ros2 launch go2_launcher dog.launch.py collect_realsense:=true" driver)
    [[[sensors]]]
      type = Terminal
      parent = top
      order = 1
      command = $(pane 2 "ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py" sensors)
    [[[slam]]]
      type = Terminal
      parent = bottom
      order = 0
      command = $(pane 20 "ros2 launch cpsl_nav slam3d.launch.py" "3D SLAM")
    [[[rviz]]]
      type = Terminal
      parent = bottom
      order = 1
      command = $(pane 25 "rviz2 -d $RVIZ" RViz)
[plugins]
EOF

cd "$WS"                                   # the YOLO node loads yolov8n.engine from here
terminator -u -g "$CFG" -l slam3d >/dev/null 2>&1 &
disown
echo "Started: driver and sensors now, 3D SLAM in 20 s, RViz in 25 s (Terminator window: Go2 3D SLAM)."
