#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <fstream>
#include <cstdlib>  // Header for getenv

// Global variables to store the parameter values
std::string filename_odometry;
std::string save_Directory;

// Global variables to store the file streams
std::ofstream odometryFile;

// Callback function to handle received odometry data
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Write the odometry data to the file
    odometryFile << msg->header.stamp << " " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << std::endl;
}

int main(int argc, char** argv) {
    // Initialize the ROS node, "odometry_subscriber" is the name of the node
    ros::init(argc, argv, "odometry_subscriber");
    // Create a ROS node handle for communication with the ROS system
    ros::NodeHandle nh_odometry;
    ros::NodeHandle nh("~");

    // Get the parameter values from the parameter server
    nh.param<std::string>("/filename_odometry", filename_odometry, "");
    nh.param<std::string>("/save_Directory", save_Directory, "");

    // Check if the save directory parameter is set
    if (save_Directory.empty()) {
        ROS_ERROR("The 'save_directory' parameter is not set. Using the default directory.");
        save_Directory = std::string(getenv("HOME")) + "/Projects/uslam_ws/src/rpg_ultimate_slam_open/data/txt_data/";
    }

    // Name of the odometry topic to subscribe to
    std::string odometry_topic = "/ze_vio/odometry";

    // Open the file streams
    odometryFile.open(save_Directory + filename_odometry);

    if (!odometryFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename_odometry.c_str());
        return -1;
    }

    // Create a ROS subscriber that listens to the odometry topic and calls the callback function
    ros::Subscriber sub_odometry = nh_odometry.subscribe(odometry_topic, 10, odometryCallback);

    // Keep the node running
    // Multithreaded spinner
    ros::MultiThreadedSpinner spinner(2);

    // Start spinning and processing callbacks
    spinner.spin(); 

    // Close the file
    odometryFile.close();
    
    return 0;
}

