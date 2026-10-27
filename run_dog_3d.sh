#!/bin/bash

SESSION="robot_session"

# 1. Start tmux session and create Pane 0
tmux new-session -d -s $SESSION -n "robot"

# 2. Create the layout by splitting panes FIRST
# Split pane 0 horizontally (Creates Pane 1 on the right)
tmux split-window -h -t $SESSION:0.0

# Split the newly created right pane vertically (Creates Pane 2)
tmux split-window -v -t $SESSION:0.1

# Split that bottom pane vertically again (Creates Pane 3)
tmux split-window -v -t $SESSION:0.2

# 3. Optional: Resize Pane 0 immediately so layouts look good
tmux select-pane -t $SESSION:0.0
tmux resize-pane -R 20

# 4. Send commands to the specific panes using exact IDs
# (Adding a small sleep ensures target shells are fully ready)
sleep 0.5 

tmux send-keys -t $SESSION:0.0 "dog_launch collect_realsense:=true" C-m
tmux send-keys -t $SESSION:0.1 "sensors_launch" C-m
tmux send-keys -t $SESSION:0.2 "slam3d_launch" C-m
tmux send-keys -t $SESSION:0.3 "rviz2" C-m

# 5. Attach so you see all logs
tmux attach-session -t $SESSION