#!/usr/bin/env zsh

########################################################################

# INITIALIZATION

# PATHS
#SH_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/scripts"
SH_PATH=$(rospack find ze_vio_ceres)/scripts
#CONF_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/cfg/dvx_parameters_study.conf"
CONF_PATH=$(rospack find ze_vio_ceres)/cfg/dvx_parameters_study.conf

# Number of iterations
iterations_number=6

# Parameters to change
declare -a param=(
imp_detector_grid_size 64 64 64 64 64 64
#imp_detector_border_margin 5 5 5 5
imp_detector_threshold 30 30 30 30 30 30
imp_detector_max_features_per_frame 300 300 300 300 300 300
#data_interval_between_event_packets 30 30 30 30 30 30
data_size_augmented_event_packet 50000 50000 50000 50000 50000 50000
vio_frame_size                   60000 60000 70000 80000 90000 90000
#vio_frame_norm_factor 1.0 2.0 3.0 4.0 # el 4 mejor
#vio_kfselect_numfts_lower_thresh 30 30 30 30
#vio_add_every_nth_frame_to_backend 2 1 1 2 # el 1 mejor
#vio_ceres_iterations 1 2 3 4 # el 1 mejor
#vio_do_motion_correction 0 1 0 1
#vio_feature_tracker_patch_size_by8 3.2 3,2 5,5 10
#vio_feature_tracker_termcrit_min_update_squared 0.1 0.2 0.5 1
#vio_viz_level 0 1 1 1 1 0
)

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
