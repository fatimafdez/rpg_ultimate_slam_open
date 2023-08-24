#!/usr/bin/env zsh
# Function to split tmux terminal into 4, 6, 8, 16, 32, .. panes
# --------------------------------------------------------------
tsplit() {
    # if [ $1 -lt 4 ]; then
    #     return
    # fi
    if [ $1 -eq 4 ]; then
        tmux split-window -h
        tmux split-window -v
        tmux select-pane -L
        #tmux split-window -v
        return
    fi
    if [ $1 -eq 6 ]; then
        tmux split-window -h -p 50
        tmux split-window -h -p 50
        tmux split-window -v
        tmux select-pane -R
        tmux split-window -v
        tmux select-pane -R
        tmux split-window -v
        tmux split-window -v
        return
    fi
    log2_odd_even=$(($(echo "l($1)/l(2)" | bc -l | xargs printf '%.0f') % 2))
    if [ $log2_odd_even -eq 0 ]; then
        tmux split-window -v
        tsplit $(($1 / 2))
        tmux select-pane -U
        tmux select-pane -U
        tsplit $(($1 / 2))
    else
        tmux split-window -h
        tsplit $(($1 / 2))
        tmux select-pane -L
        tsplit $(($1 / 2))
    fi
}
# --------------------------------------------------------------
# Set Session Name
SESSION="parameters_uslam"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)
LOCAL_IP='127.0.0.1'


# PATHS
WS_PATH="$HOME/Projects/uslam_ws"
SH_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/scripts"
BAGS_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/data"

# FILE NAMES
BAG_NAME="uslam_01_x.bag"

# Source of ROS workspace
SOURCE_ROUTE1="$HOME/Projects/uslam_ws/devel/setup.zsh"


# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" != "" ]; then
    echo Destroying last session
    tmux kill-session -t $SESSION
    SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)
fi
if [ "$SESSIONEXISTS" = "" ]; then
    echo Creating new session
    # Start New Session with our name
    tmux new-session -d -s $SESSION
    tsplit 4
    tmux setw synchronize-panes on
    tmux send-keys -t $SESSION "export ROS_MASTER_URI=http://$LOCAL_IP:11311" C-m
    tmux send-keys -t $SESSION "export ROS_IP=$LOCAL_IP" C-m
    tmux send-keys -t $SESSION "tput reset" C-m
    tmux setw synchronize-panes off


    tmux send-keys -t $SESSION:0.1 "roscore" C-m

    tmux send-keys -t $SESSION:0.0 "cd $WS_PATH && source $SOURCE_ROUTE1 && roslaunch ze_vio_ceres live_DVXplorer.launch --wait" C-m

    tmux send-keys -t $SESSION:0.2 "cd $BAGS_PATH && rosbag play -d 2 $BAG_NAME use_sim_time:=true --clock --wait && print termine" C-m

    # tmux send-keys -t $SESSION:0.2 "source $SOURCE_ROUTE4" C-m
    # docker container exec -it dvx_uslam zsh
fi
# Attach Session, on the Main window
tmux attach-session -t $SESSION
exit 0

# docker container exec -it uslam_base_container zsh