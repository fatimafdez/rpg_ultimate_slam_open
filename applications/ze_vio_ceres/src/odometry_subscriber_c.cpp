#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <fstream>
#include <cstdlib>  // Header for getenv

// Default parameters
// Default txt file names
std::string filename_odometry = "uslam_odometry_1.txt";
std::string filename_vicon = "uslam_vicon_1.txt";

// Default vicon topic name
std::string viconTopicName = "/vicon_client/dvxplorer/pose";

// Save directory
std::string save_Directory= std::string(getenv("HOME")) + "/Projects/uslam_ws/src/rpg_ultimate_slam_open/data/txt_data/";

// Global variable to store the file stream
std::ofstream odometryFile;
std::ofstream viconFile;

// Callback function to handle received odometry data
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Access and process the odometry data here
    //ROS_INFO("Received Odometry Data:\nPosition: x=%.2f, y=%.2f, z=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

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
    ros::init(argc, argv, "vicon_subscriber");
    // Create a ROS node handle for communication with the ROS system
    ros::NodeHandle nh_odometry;
    ros::NodeHandle nh_vicon;

    // Check if arguments were provided for file names and vicon topic
    if (argc >= 4) {
        filename_odometry = argv[1];
        filename_vicon = argv[2];
        viconTopicName = argv[3];
    } else {
        ROS_WARN("Using default values for parameters. Correct usage: rosrun ze_vio_ceres tmux_txt_writing_automatization.sh <odometry_file> <vicon_file> <vicon_topic>");
        return 1;
    }

    // Name of the odometry topic to subscribe to
    std::string odometry_topic = "/ze_vio/odometry";
    std::string viconTopicParam;
    ros::param::get("/vicon_topic", viconTopicParam);
    std::string vicon_topic = viconTopicParam.empty() ? viconTopicName : viconTopicParam;

    ros::param::param<std::string>("/save_directory", save_Directory);

    filename_odometry = save_Directory + filename_odometry;
    filename_vicon = save_Directory + filename_vicon;

    // Create a ROS subscriber that listens to the odometry topic and calls the callback function
    ros::Subscriber sub_odometry = nh_odometry.subscribe(odometry_topic, 1, odometryCallback);
    ros::Subscriber sub_vicon = nh_vicon.subscribe(vicon_topic, 1, viconCallback);
    // queue size = 1, i.e. only the latest message is stored in the queue

    // Open file to write odometry data
    odometryFile.open(filename_odometry.c_str());
    viconFile.open(filename_vicon.c_str(), std::ios::app);

    if (!odometryFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename_odometry.c_str());
        return -1;
    }

    if (!viconFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename_vicon.c_str());
        return -1;
    }

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

