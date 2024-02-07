#!/bin/zsh

########################################################################
# PARAMETERS TO BE SET
########################################################################

# Global parameters
# Source of ROS workspace
export SOURCE_ROUTE1=$HOME/Projects/uslam_ws/devel/setup.zsh
# Workspace path
export WS_PATH=$HOME/Projects/uslam_ws
# Bags path
export BAGS_PATH=$HOME/Utils/testbed_events/f550_events_vuelos_ir
# Bag name
# export BAG_NAME=f550_events_2023-10-05-12-40-41.bag
export BAG_NAME=f550_events_2023-10-05-12-46-29.bag
export time_initialization=30

########################################################################
# WHAT ARE WE USING?
########################################################################

# If the bag that want to be executed uses the Vicon topic, then the next parameter must be true:
export use_vicon=true
# also you have to set the vicon topic name:
export vicon_topic_name=/vicon_client/f550_events/pose

# If you want to write the data in a txt file, then the next parameter must be true:
export write_txt_vicon=true
export write_txt_odometry=true
# also you have to set the directory where the txt files will be saved:
export save_Directory=$HOME/Utils/testbed_events/txt_writing/
# and the name of the txt files:
export filename_odometry=uslam_30_f550_events_2023-10-05-12-46-29-odometry.txt
# if you have vicon also set the name of the vicon txt file:
export filename_vicon=uslam_30_f550_events_2023-10-05-12-46-29-vicon.txt


########################################################################
# ODOMETRY PARAMETERS
########################################################################

# Calibration file
# when using camera imu and no ir filter 'DVXplorer.yaml'
# when using camera imu and ir filter 'DVXplorer-ir.yaml'
# when using autopilot imu and ir filter 'DVXplorer-ir-autopiloto'
export calibration_camera_filename=$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/calibration/DVXplorer-ir.yaml
    
# IMU calibration file
# when using camera imu 'dvx_imu_parameters'
# when using autopilot imu 'autopiloto_imu_parameters'
export calibration_imu_filename=$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/cfg/dvx_imu_parameters.conf

# USLAM parameters
# f we want to use only one parameter 'dvx_parameters.yaml'
# f we want to do a parameters study 'dvx_parameters_study.yaml'
export odometry_parameters_filename=$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/cfg/dvx_parameters_study.conf

########################################################################
# SCRIPT TO BE EXECUTED
########################################################################

/$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/scripts/tmux_parameters_automatization.sh
# The parameters_automatization.sh script will iterate some values
#/$HOME/Projects/uslam_ws/src/rpg_ultimate_slam_open/applications/ze_vio_ceres/scripts/parameters_automatization.sh



# FILE NAMES
# BAG_NAME=2023-07-04-12-51.bag # BAG_NAME=2023-07-04-12-53.bag
# BAG_NAME=events_F550_03-16-11.bag
# BAG_NAME=uslam_vicon_00.bag # BAG_NAME=uslam_vicon_01.bag
# BAG_NAME=uslam_vicon_0.bag # BAG_NAME=uslam_vicon_1.bag # BAG_NAME=uslam_vicon_2.bag # BAG_NAME=uslam_vicon_3.bag # BAG_NAME=uslam_vicon_4.bag
# BAG_NAME=uslam_vicon_11.bag # BAG_NAME=uslam_vicon_12.bag
# BAG_NAME=f550_events_2023-10-04-13-11-38.bag # BAG_NAME=f550_events_2023-10-04-13-17-32.bag
# BAG_NAME=f550_events_2023-10-04-13-21-55.bag # BAG_NAME=f550_events_2023-10-04-13-25-00.bag
# BAG_NAME=f550_events_2023-10-04-13-27-18.bag # BAG_NAME=f550_events_2023-10-04-16-25-55.bag # BAG_NAME=f550_events_2023-10-04-16-31-40.bag

# VUELOS
# BAG_NAME=f550_events_2023-10-05-12-22-06.bag
# BAG_NAME=f550_events_2023-10-05-12-24-20.bag 
# BAG_NAME=f550_events_2023-10-05-12-40-41.bag
# BAG_NAME=f550_events_2023-10-05-12-46-29.bag
# BAG_NAME=f550_events_2023-10-05-12-55-57.bag
# BAG_NAME=f550_events_2023-10-05-12-59-27.bag
# BAG_NAME=f550_events_2023-10-05-13-12-02.bag

