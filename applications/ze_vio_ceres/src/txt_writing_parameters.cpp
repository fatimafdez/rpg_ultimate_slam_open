#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <fstream>
#include <cstdlib>  // Header for getenv

// Global variables to store the parameter values
std::string filename_odometry;
std::string filename_vicon;
std::string vicon_topic_name;
std::string save_Directory;

// Global variables to store the file streams
std::ofstream odometryFile;
std::ofstream viconFile;

// Callback function to handle received odometry data
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Write the odometry data to the file
    odometryFile << msg->header.stamp << " " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << std::endl;
}

// Callback function to handle received vicon data
void viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // Write the vicon data to the file
    viconFile << msg->header.stamp << " " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << std::endl;   
}

int main(int argc, char** argv) {
    // Initialize the ROS node, "odometry_subscriber" is the name of the node
    ros::init(argc, argv, "odometry_subscriber");
    // Create a ROS node handle for communication with the ROS system
    ros::NodeHandle nh_odometry;
    ros::NodeHandle nh("~");

    std::string filename_odometry, filename_vicon, vicon_topic_name, save_Directory;
    bool write_txt_odometry, write_txt_vicon;
    // Name of the odometry topic to subscribe to
    std::string odometry_topic = "/ze_vio/odometry";

    // Get the parameter values from the parameter server
    nh.param<bool>("/write_txt_odometry", write_txt_odometry, false);
    nh.param<bool>("/write_txt_vicon", write_txt_vicon, false);
    nh.param<std::string>("/filename_odometry", filename_odometry, "");
    nh.param<std::string>("/filename_vicon", filename_vicon, "");
    nh.param<std::string>("/vicon_topic_name", vicon_topic_name, "");
    nh.param<std::string>("/save_Directory", save_Directory, "");

    // Print the parameter values
    ROS_INFO_STREAM("filename_odometry: " << filename_odometry);
    ROS_INFO_STREAM("filename_vicon: " << filename_vicon);
    ROS_INFO_STREAM("vicon_topic_name: " << vicon_topic_name);
    ROS_INFO_STREAM("vicon_topic_name: " << vicon_topic_name);
    ROS_INFO_STREAM("save_Directory: " << save_Directory);

    // Check if the save directory parameter is set
    if (save_Directory.empty()) {
        ROS_ERROR("The 'save_directory' parameter is not set.");
        return -1;
    }

    if (write_txt_vicon) {
    viconFile.open(save_Directory + filename_vicon);
    }

    if (!viconFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename_vicon.c_str());
        //return -1;
    }
    ros::Subscriber sub_vicon = nh.subscribe<geometry_msgs::PoseStamped>(vicon_topic_name, 10, viconCallback);

    if (write_txt_odometry) {
    // Open the file streams
    odometryFile.open(save_Directory + filename_odometry);
    }

    if (!odometryFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename_odometry.c_str());
        //return -1;
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
    viconFile.close();
    
    return 0;
}

