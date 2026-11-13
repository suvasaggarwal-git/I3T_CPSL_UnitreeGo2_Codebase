#!/bin/bash
# A COMRES mission from this dog, in one Terminator window on its desktop (e.g. through NoMachine):
#
#   comres_explore [target class(es)]               explore, with no prior information
#   comres_search "<hint>" [target class(es)]       search, with the prior in plain words, e.g.
#       comres_search "The backpack is in the room north of the start." backpack
#
#   driver (dog.launch.py)       | sensors (Livox)
#   3D SLAM (RTAB-Map), +20 s    | Nav2, +35 s
#   RViz (maps and YOLO), +25 s  | COMRES bridge, +40 s
#
# The bridge (~/comres_bridge: dog_bridge.py, dog_wire.py and comres_bt.xml from COMRES-VLM-ROS)
# asks the central computer for the mission once Nav2 is up and there is a map; the COMRES node
# there (tools/comres_central.sh) starts it. Without a target class, the central config's is used
# (search ends when the YOLO sees it). Ctrl-C in a pane stops that part; closing the window stops
# everything, and the bridge then cancels the robot's goals.

WS="$HOME/I3T_CPSL_UnitreeGo2_codebase"
BRIDGE="${COMRES_BRIDGE:-$HOME/comres_bridge}"
RVIZ="$WS/install/cpsl_nav/share/cpsl_nav/rviz_cfgs/slam_3d_yolo.rviz"
CFG="${XDG_RUNTIME_DIR:-/tmp}/comres.terminator"

case "$1" in
    explore) export COMRES_TASK=explore COMRES_HINT="" COMRES_TARGETS="$2" ;;
    search)
        if [ $# -lt 2 ]; then
            echo "usage: comres_search \"<hint>\" [target class(es)]" >&2
            exit 1
        fi
        export COMRES_TASK=search COMRES_HINT="$2" COMRES_TARGETS="$3" ;;
    *)
        echo "usage: $0 explore [target class(es)] | search \"<hint>\" [target class(es)]" >&2
        exit 1 ;;
esac
export COMRES_ROBOT="${COMRES_ROBOT:-go2_1}"

if [ -z "$DISPLAY" ]; then
    echo "No display: run this on the dog's desktop (e.g. through NoMachine)." >&2
    exit 1
fi
if [ ! -f "$BRIDGE/dog_bridge.py" ]; then
    echo "No $BRIDGE/dog_bridge.py: copy dog_bridge.py, dog_wire.py and comres_bt.xml there from COMRES-VLM-ROS." >&2
    exit 1
fi
RUNNING="bin/ros2 launch|lib/rviz2/rviz2|dog_bridge.py"
if pgrep -f "$RUNNING" >/dev/null; then
    echo "ROS is already running on this dog:" >&2
    pgrep -af "$RUNNING" | cut -c1-120 >&2
    echo "Close its window first, or: pkill -INT -f '$RUNNING'" >&2
    exit 1
fi

# The panes inherit this environment (as ~/.bashrc sets it up) and the mission above.
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
  [[comres]]
    [[[window0]]]
      type = Window
      parent = ""
      title = Go2 COMRES $COMRES_TASK
      size = 1500, 1000
    [[[root]]]
      type = VPaned
      parent = window0
      order = 0
    [[[top]]]
      type = HPaned
      parent = root
      order = 0
    [[[rest]]]
      type = VPaned
      parent = root
      order = 1
    [[[middle]]]
      type = HPaned
      parent = rest
      order = 0
    [[[bottom]]]
      type = HPaned
      parent = rest
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
      parent = middle
      order = 0
      command = $(pane 20 "ros2 launch cpsl_nav slam3d.launch.py" "3D SLAM")
    [[[nav2]]]
      type = Terminal
      parent = middle
      order = 1
      command = $(pane 35 "ros2 launch cpsl_nav nav2_go2.launch.py" Nav2)
    [[[rviz]]]
      type = Terminal
      parent = bottom
      order = 0
      command = $(pane 25 "rviz2 -d $RVIZ" RViz)
    [[[bridge]]]
      type = Terminal
      parent = bottom
      order = 1
      command = $(pane 40 "python3 $BRIDGE/dog_bridge.py --robot $COMRES_ROBOT" "COMRES bridge")
[plugins]
EOF

cd "$WS"                                   # the YOLO node loads yolov8n.engine from here
terminator -u -g "$CFG" -l comres >/dev/null 2>&1 &
disown
echo "Started a $COMRES_TASK mission for $COMRES_ROBOT${COMRES_HINT:+, hint: $COMRES_HINT}${COMRES_TARGETS:+, targets: $COMRES_TARGETS}."
echo "Driver and sensors now, 3D SLAM in 20 s, Nav2 in 35 s, the bridge in 40 s; COMRES starts when the dog is ready."
