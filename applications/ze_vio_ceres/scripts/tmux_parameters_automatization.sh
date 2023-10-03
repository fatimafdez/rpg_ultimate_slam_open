#!/usr/bin/env zsh

# Create ~/.tmux.conf if it does not exist
# --------------------------------------------------------------
if [ ! -f ~/.tmux.conf ]; then
    touch ~/.tmux.conf
    echo "set -g default-terminal 'screen-256color'" >~/.tmux.conf
    echo "set -g status-bg green" >>~/.tmux.conf
    echo "set -g status-right \"%H:%M\"" >>~/.tmux.conf
    echo "set -g history-limit 1000000" >>~/.tmux.conf
    echo "set -g mouse on" >>~/.tmux.conf
fi

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

########################################################################
# PARAMETERS TO BE SET
########################################################################

# PATHS
WS_PATH="$HOME/Projects/uslam_ws"
BAGS_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/data"

# FILE NAMES
# BAG_NAME=2023-07-04-12-51.bag
# BAG_NAME=2023-07-04-12-53.bag
# BAG_NAME=events_F550_03-16-11.bag
# BAG_NAME=uslam_01_x.bag
# BAG_NAME=uslam_01_y.bag
# BAG_NAME=uslam_01_z.bag
# BAG_NAME=uslam_vicon_00.bag
# BAG_NAME=uslam_vicon_01.bag
# BAG_NAME=uslam_vicon_0.bag
# BAG_NAME="uslam_vicon_1.bag"
# BAG_NAME=uslam_vicon_2.bag
# BAG_NAME=uslam_vicon_3.bag
# BAG_NAME=uslam_vicon_4.bag
 BAG_NAME="uslam_vicon_11.bag"
# BAG_NAME=uslam_vicon_12.bag

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

    tmux send-keys -t $SESSION:0.0 "cd $WS_PATH && source $SOURCE_ROUTE1 && roslaunch ze_vio_ceres live_DVXplorer_study.launch --wait" C-m

    tmux send-keys -t $SESSION:0.2 "cd $BAGS_PATH && rosbag play -d 2 $BAG_NAME use_sim_time:=true --clock -s 24 --wait && print termine" C-m

fi
# Attach Session, on the Main window
tmux attach-session -t $SESSION
exit 0