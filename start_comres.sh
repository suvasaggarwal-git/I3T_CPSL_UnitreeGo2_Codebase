#!/bin/bash
# COMRES missions from this dog, in Terminator windows on its desktop (e.g. through NoMachine).
#
# In two steps (aliases comres_explore_prep / comres_search_prep, then comres_explore_start /
# comres_search_start):
#   start_comres.sh prep                            driver, sensors, 3D SLAM (+20 s), RViz (+25 s):
#                                                   drive the dog by hand for a first map
#   start_comres.sh start explore [class]           then, beside it: Nav2 and the COMRES bridge,
#   start_comres.sh start search "<hint>" [class]   which asks for the mission; COMRES plans on the
#                                                   map so far and sends the dog its waypoints
#       e.g. comres_search_start "The backpack is in the room north of the start." backpack
# At once: start_comres.sh explore [class]  |  start_comres.sh search "<hint>" [class]
#
# The bridge (~/comres_bridge: dog_bridge.py, dog_wire.py and comres_bt.xml from COMRES-VLM-ROS)
# asks the central computer (tools/comres_central.sh) for the mission once Nav2 is up and there is
# a map; COMRES starts it there. Without a target class, the central config's is used. RViz shows
# the map, the YOLO, the waypoint COMRES gave (/comres/goal) and Nav2's path. Ctrl-C in a pane
# stops that part; closing a window stops its parts (the bridge then cancels the robot's goals).

WS="$HOME/I3T_CPSL_UnitreeGo2_codebase"
BRIDGE="${COMRES_BRIDGE:-$HOME/comres_bridge}"
RVIZ="$WS/install/cpsl_nav/share/cpsl_nav/rviz_cfgs/slam_3d_yolo.rviz"
CFG="${XDG_RUNTIME_DIR:-/tmp}/comres-$$.terminator"
USAGE="usage: $0 prep | start explore [class] | start search \"<hint>\" [class] | explore [class] | search \"<hint>\" [class]"

STEP=all
case "$1" in
    prep) STEP=prep ;;
    start) STEP=start; shift ;;
esac
if [ "$STEP" != prep ]; then
    case "$1" in
        explore) export COMRES_TASK=explore COMRES_HINT="" COMRES_TARGETS="$2" ;;
        search)
            [ $# -ge 2 ] || { echo "$USAGE" >&2; exit 1; }
            export COMRES_TASK=search COMRES_HINT="$2" COMRES_TARGETS="$3" ;;
        *) echo "$USAGE" >&2; exit 1 ;;
    esac
fi
export COMRES_ROBOT="${COMRES_ROBOT:-go2_1}"

if [ -z "$DISPLAY" ]; then
    echo "No display: run this on the dog's desktop (e.g. through NoMachine)." >&2
    exit 1
fi
if [ "$STEP" != prep ] && [ ! -f "$BRIDGE/dog_bridge.py" ]; then
    echo "No $BRIDGE/dog_bridge.py: copy dog_bridge.py, dog_wire.py and comres_bt.xml there from COMRES-VLM-ROS." >&2
    exit 1
fi
running() { pgrep -f "$1" >/dev/null; }
if [ "$STEP" = start ]; then
    if ! running "go2_launcher.*dog.launch.py" || ! running "cpsl_nav.*slam3d.launch.py"; then
        echo "The driver and the 3D SLAM are not running: comres_${COMRES_TASK}_prep first." >&2
        exit 1
    fi
    if running "nav2_go2.launch.py|dog_bridge.py"; then
        echo "Nav2 or the COMRES bridge already runs: close their window first." >&2
        exit 1
    fi
else
    RUNNING="bin/ros2 launch|lib/rviz2/rviz2|dog_bridge.py"
    if running "$RUNNING"; then
        echo "ROS is already running on this dog:" >&2
        pgrep -af "$RUNNING" | cut -c1-120 >&2
        echo "Close its window first, or: pkill -INT -f '$RUNNING'" >&2
        exit 1
    fi
fi

# The panes inherit this environment (as ~/.bashrc sets it up) and the mission above.
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/unitree_msgs_ws/install/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml

# The panes, "delay|title|command" (no commas, quotes or |: they would break Terminator's config).
DRIVER="0|driver|ros2 launch go2_launcher dog.launch.py collect_realsense:=true"
SENSORS="2|sensors|ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py"
SLAM="20|3D SLAM|ros2 launch cpsl_nav slam3d.launch.py"
SHOW="25|RViz|rviz2 -d $RVIZ"
case "$STEP" in
    prep)  TITLE="Go2 COMRES prep: drive by hand"
           PANES=("$DRIVER" "$SENSORS" "$SLAM" "$SHOW") ;;
    start) TITLE="Go2 COMRES $COMRES_TASK"
           PANES=("0|Nav2|ros2 launch cpsl_nav nav2_go2.launch.py"
                  "5|COMRES bridge|python3 $BRIDGE/dog_bridge.py --robot $COMRES_ROBOT") ;;
    all)   TITLE="Go2 COMRES $COMRES_TASK"
           PANES=("$DRIVER" "$SENSORS" "$SLAM" "35|Nav2|ros2 launch cpsl_nav nav2_go2.launch.py" "$SHOW"
                  "40|COMRES bridge|python3 $BRIDGE/dog_bridge.py --robot $COMRES_ROBOT") ;;
esac

# A pane's command: wait, run, then keep the pane open with a shell.
pane() {
    IFS='|' read -r delay title cmd <<< "$1"
    echo "sleep $delay; $cmd; echo; echo [$title ended - Ctrl-D closes this pane]; exec bash"
}
# Two panes a row: the rows nest in VPaneds, a row of two is an HPaned.
layout() {
    local n=${#PANES[@]} rows=$(( (${#PANES[@]} + 1) / 2 )) parent=window0 order=0 r rp ro
    printf '[global_config]\n  suppress_multiple_term_dialog = True\n[keybindings]\n[profiles]\n'
    printf '  [[default]]\n    scrollback_lines = 10000\n[layouts]\n  [[comres]]\n'
    printf '    [[[window0]]]\n      type = Window\n      parent = ""\n      title = %s\n      size = 1500, %d\n' "$TITLE" $((330 * rows))
    for ((r = 0; r < rows; r++)); do
        rp=$parent; ro=$order
        if ((r < rows - 1)); then
            printf '    [[[v%d]]]\n      type = VPaned\n      parent = %s\n      order = %d\n' $r "$parent" $order
            rp=v$r; ro=0; parent=v$r; order=1
        fi
        if ((2 * r + 1 < n)); then
            printf '    [[[h%d]]]\n      type = HPaned\n      parent = %s\n      order = %d\n' $r "$rp" $ro
            printf '    [[[t%d]]]\n      type = Terminal\n      parent = h%d\n      order = 0\n      command = %s\n' $((2 * r)) $r "$(pane "${PANES[2 * r]}")"
            printf '    [[[t%d]]]\n      type = Terminal\n      parent = h%d\n      order = 1\n      command = %s\n' $((2 * r + 1)) $r "$(pane "${PANES[2 * r + 1]}")"
        else
            printf '    [[[t%d]]]\n      type = Terminal\n      parent = %s\n      order = %d\n      command = %s\n' $((2 * r)) "$rp" $ro "$(pane "${PANES[2 * r]}")"
        fi
    done
    printf '[plugins]\n'
}
layout > "$CFG"

cd "$WS"                                   # the YOLO node loads yolov8n.engine from here
terminator -u -g "$CFG" -l comres >/dev/null 2>&1 &
disown
case "$STEP" in
    prep)  echo "Started the prep: driver and sensors now, 3D SLAM in 20 s, RViz in 25 s. Drive the dog by hand"
           echo "for a first map, then: comres_explore_start, or comres_search_start \"<hint>\" [class]" ;;
    start) echo "Started Nav2 and the COMRES bridge for a $COMRES_TASK mission${COMRES_HINT:+, hint: $COMRES_HINT}${COMRES_TARGETS:+, targets: $COMRES_TARGETS}."
           echo "COMRES starts when Nav2 is up (the central computer must run tools/comres_central.sh)." ;;
    all)   echo "Started a $COMRES_TASK mission for $COMRES_ROBOT${COMRES_HINT:+, hint: $COMRES_HINT}${COMRES_TARGETS:+, targets: $COMRES_TARGETS}."
           echo "Driver and sensors now, 3D SLAM in 20 s, Nav2 in 35 s, the bridge in 40 s; COMRES starts when the dog is ready." ;;
esac
