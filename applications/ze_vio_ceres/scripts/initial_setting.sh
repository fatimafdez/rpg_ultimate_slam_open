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
export BAGS_PATH=$HOME/Utils/testbed_events/f550_events_manual_ir
# export BAGS_PATH=$HOME/Utils/testbed_events/f550_events_vuelos_ir
# export BAGS_PATH=$HOME/Utils/testbed_events

# Bag name
# Bags names - manual - ir
# export BAG_NAME=f550_events_2023-10-04-13-11-38.bag # -
# export BAG_NAME=f550_events_2023-10-04-13-17-32.bag # 
# export BAG_NAME=f550_events_2023-10-04-13-21-55.bag # ! -(Exp3)- initial time 42 - random movement - yaw rotation - height changes
# export BAG_NAME=f550_events_2023-10-04-13-25-00.bag #
# export BAG_NAME=f550_events_2023-10-04-13-27-18.bag #
# export BAG_NAME=f550_events_2023-10-04-16-25-55.bag # ! -(Exp1)- initial time 88 - square trajectory - no yaw rotation
# export BAG_NAME=f550_events_2023-10-04-16-31-40.bag # ! -(Exp2)- initial time 0  - square trajectory(?) - arucos
export BAG_NAME=f550_events_2023-10-09-16-30-30.bag # ! -(Exp4)- initial time 12 - square trajectory - ilumination changes

# VUELOS IR
# export BAG_NAME=f550_events_2023-10-05-12-22-06.bag
# export BAG_NAME=f550_events_2023-10-05-12-24-20.bag 
# export BAG_NAME=f550_events_2023-10-05-12-40-41.bag # ! -(Exp5)- initial time 80 - square trajectory - no yaw rotation - best case -
# export BAG_NAME=f550_events_2023-10-05-12-46-29.bag # ! -(Exp6)- initial time 30 - square trajectory - yaw rotation 
# export BAG_NAME=f550_events_2023-10-05-12-55-57.bag #
# export BAG_NAME=f550_events_2023-10-05-12-59-27.bag #
# export BAG_NAME=f550_events_2023-10-05-13-12-02.bag # ! -(Exp7)- initial time 20 - random movement - fast

export time_initialization=12

########################################################################
# WHAT ARE WE USING?
########################################################################

# If the bag that want to be executed uses the Vicon topic, then the next parameter must be true:
export use_vicon=false
# also you have to set the vicon topic name:
export vicon_topic_name=/vicon_client/f550_events/pose

# If you want to write the data in a txt file, then the next parameter must be true:
export write_txt_vicon=false
export write_txt_odometry=false
# also you have to set the directory where the txt files will be saved:
export save_Directory=$HOME/Utils/testbed_events/txt_writing/
# and the name of the txt files:
export filename_odometry=try_odometry.txt
# if you have vicon also set the name of the vicon txt file:
export filename_vicon=try_vicon.txt


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


# FILE NAMES - manual - sin ir
# BAG_NAME=2023-07-04-12-51.bag 
# BAG_NAME=2023-07-04-12-53.bag
# BAG_NAME=uslam_vicon_00.bag # BAG_NAME=uslam_vicon_01.bag
# BAG_NAME=uslam_vicon_0.bag # BAG_NAME=uslam_vicon_1.bag 
# BAG_NAME=uslam_vicon_2.bag # BAG_NAME=uslam_vicon_3.bag 
# BAG_NAME=uslam_vicon_4.bag
# BAG_NAME=uslam_vicon_11.bag # BAG_NAME=uslam_vicon_12.bag


# Bags - vuelo - sin ir
# BAG_NAME=events_F550_03-16-11-39-20.bag
