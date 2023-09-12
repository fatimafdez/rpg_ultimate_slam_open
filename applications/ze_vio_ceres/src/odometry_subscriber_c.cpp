#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <fstream>

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

    // Name of the odometry topic to subscribe to
    std::string odometry_topic = "/ze_vio/odometry";
    std::string vicon_topic = "/vicon_client/dvxplorer02/pose";

    // Create a ROS subscriber that listens to the odometry topic and calls the callback function
    ros::Subscriber sub_odometry = nh_odometry.subscribe(odometry_topic, 1, odometryCallback);
    // queue size = 1, i.e. only the latest message is stored in the queue
    ros::Subscriber sub_vicon = nh_vicon.subscribe(vicon_topic, 1, viconCallback);

    // Name of the output file
    std::string filename="uslam_trajectory.txt";
    std::string filename_odometry="uslam_odometry.txt";
    std::string filename_vicon="uslam_vicon.txt";

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

