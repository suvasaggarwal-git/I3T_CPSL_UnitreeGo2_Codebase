#!/bin/bash

SESSION="robot_session"

# Start tmux session
tmux new-session -d -s $SESSION -n "robot"

###############################################
# Pane 1: dog_launch (left)
###############################################
tmux send-keys -t $SESSION:0.0 "dog_launch collect_realsense:=true" C-m

###############################################
# Pane 2: sensors_launch (top-right)
###############################################
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sensors_launch" C-m

###############################################
# Pane 3: slam_launch (bottom-right)
###############################################
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "slam3d_launch" C-m

###############################################
# Pane 4: RViz (bottom-far-right)
###############################################
tmux split-window -v -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 "rviz2" C-m

###############################################
# Optional: resize panes for readability
###############################################
tmux select-pane -t $SESSION:0.0
tmux resize-pane -R 20

# Attach so you see all logs
tmux attach-session -t $SESSION