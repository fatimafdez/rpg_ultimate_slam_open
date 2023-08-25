#!/usr/bin/env zsh

########################################################################

# INITIALIZATION

# PATHS
SH_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/scripts"
CONF_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/cfg/dvx_parameters_study.conf"

# Source of ROS workspace
SOURCE_ROUTE1="$HOME/Projects/uslam_ws/devel/setup.zsh"

# Number of iterations
iterations_number=3

# Parameters to change
declare -a param=(imp_detector_grid_size 32 64 128
imp_detector_name FAST FAST FAST
imp_detector_border_margin 5 10 20
imp_detector_threshold 50 50 50
imp_detector_max_features_per_frame 100 100 100)

########################################################################

# Source the workspace
source $SOURCE_ROUTE1

# Initialization
matrix_length=0
parameters_number=0
var_aux=1

matrix_length=${#param[@]}
parameters_number=${#param[@]}/($iterations_number+1)

for ((i = 1; i <= iterations_number; i++)); do
    for ((j = 1; j <= parameters_number; j++)); do
        param_name="${param[$var_aux]}"
        value="${param[$var_aux+$i]}"
        var_aux=$var_aux+$iterations_number+1
        
        printf "$param_name $value\n"
        # Change the value of the parameter in the configuration file
        sed -i "s/--$param_name=.*/--$param_name=$value/g" $CONF_PATH

    done
    # Reset auxiliar variable
    var_aux=1 

    cd $SH_PATH && ./tmux_parameters_automatization.sh &

    sleep 2

    # Wait for the bag in tmux to finish playing
    while true; do 
    if pgrep -x "rosbag" > /dev/null; then
        printf "rosbag is running\n"
    else
        printf "rosbag is not running\n"
        break
    fi
    sleep 3
    done 
    
    # Finish the tmux session
    tmux kill-session -t parameters_uslam

    printf "Iteration complete\n"

    sleep 1

done 

########################################################################
