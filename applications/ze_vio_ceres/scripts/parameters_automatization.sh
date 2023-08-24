#!/usr/bin/env zsh

########################################################################
# INITIALIZATION

# PATHS
WS_PATH="$HOME/Projects/uslam_ws"
SH_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/scripts"
BAGS_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/data"
CONF_PATH="$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/cfg/"

# FILE NAMES
BAG_NAME="uslam_01_x.bag"
CONF_FILE="dvx_parameters_study.conf"

# Source of ROS workspace
SOURCE_ROUTE1="$HOME/Projects/uslam_ws/devel/setup.zsh"

# Parameters values for iteration
grid_sizes=(32 64 128)
detector_names=(FAST ORB BRISK)
border_margins=(5 10 20)
thresholds=(50 50 50)
max_features=(100 100 100)

########################################################################

# Path to the workspace
cd $WS_PATH
# Source the workspace
source $SOURCE_ROUTE1

# Loop for each parameter
for ((i=0; i<${#grid_sizes[@]}; i++)); do
    grid_size=${grid_sizes[i]}
    detector_name=${detector_names[i]}
    border_margin=${border_margins[i]}
    threshold=${thresholds[i]}
    max_feature=${max_features[i]}

    # Parameters changes in configuration
    sed -i "s/--imp_detector_grid_size=(.*)/--imp_detector_grid_size=($grid_size)/g" $conf_file
    sed -i "s/--imp_detector_name=(.*)/--imp_detector_name=($detector_name)/g" $conf_file
    sed -i "s/--imp_detector_border_margin=(.*)/--imp_detector_border_margin=($border_margin)/g" $conf_file
    sed -i "s/--imp_detector_threshold=(.*)/--imp_detector_threshold=($threshold)/g" $conf_file
    sed -i "s/--imp_detector_max_features_per_frame=(.*)/--imp_detector_max_features_per_frame=($max_feature)/g" $conf_file

    cd $SH_PATH && ./tmux_parameters_automatization.sh
    
    # Wait for the bag in tmux to finish playing by reading the output of rosbag
    while [ -n "$(tmux capture-pane -p -t 0 | grep 'Done reading bag')" ]; do
        sleep 1
    done

    # Send a shutdown signal to the ze_vio_ceres launch
    rostopic pub /ze_vio_ceres/exit std_msgs/Empty -1

    # Wait
    sleep 5

    # Once the bag is finished, kill the SLAM launch
    killall -9 roslaunch
    sleep 5

done 


########################################################################



#roscore &
#sleep 5

# gnome-terminal -- zsh -c "roscore; exec zsh"
    #gnome-terminal -- zsh -c "roslaunch ze_vio_ceres live_DVXplorer.launch; exec zsh"
    #gnome-terminal -- zsh -c "rosbag play ~/Projects/uslam_ws/src/rpg_ultimate_slam_open/data/uslam_01_x.bag; exec zsh"


    # Execute the SLAM launch in the background
#     roslaunch ze_vio_ceres live_DVXplorer.launch --wait &

#     # Send a shutdown signal to the ze_vio_ceres launch
#     sleep 35
#     rostopic pub /ze_vio_ceres/exit std_msgs/Empty -1

#     # Wait for the nodes to shutdown
#     sleep 5

#     # Once the bag is finished, kill the SLAM launch
#     killall -9 roslaunch
#     sleep 5
    
# done


########################################################################

# Define associative arrays for parameters and their values
# typeset -A parameters
# parameters["imp_detector_grid_size"]="32 64 128"
# parameters["imp_detector_name"]="FAST ORB BRISK"
# parameters["imp_detector_border_margin"]="5 10 20"
# parameters["imp_detector_threshold"]="50 50 50"
# parameters["imp_detector_max_features_per_frame"]="100 100 100"


# # Loop through parameters
# for param in "${!parameters[@]}"; do
#     values=("${parameters[$param]}")
#     for value in "${values[@]}"; do
#         # Change the value of the parameter in the configuration file
#         sed -i "s/--$param=(.*)/--$param=($value)/g" $CONF_PATH$CONF_FILE

#         cd $SH_PATH && ./parameters_automatization.sh
    
#         # Wait for the bag in tmux to finish playing by reading the output of rosbag
#         while [ -n "$(tmux capture-pane -p -t 0 | grep 'Done reading bag')" ]; do
#             sleep 1
#         done

#         # Send a shutdown signal to the ze_vio_ceres launch
#         rostopic pub /ze_vio_ceres/exit std_msgs/Empty -1

#         # Wait
#         sleep 5

#         # Once the bag is finished, kill the SLAM launch
#         killall -9 roslaunch
#         sleep 5
#     done
# done 