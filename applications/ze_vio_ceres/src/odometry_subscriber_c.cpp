#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <fstream>

// Global variable to store the file stream
std::ofstream outputFile;

// Callback function to handle received odometry data
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Access and process the odometry data here
    //ROS_INFO("Received Odometry Data:\nPosition: x=%.2f, y=%.2f, z=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // Write the odometry data to the file
    outputFile << msg->header.stamp << " " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << std::endl;
    
}


int main(int argc, char** argv) {
    // Initialize the ROS node, "odometry_subscriber" is the name of the node
    ros::init(argc, argv, "odometry_subscriber");

    // Create a ROS node handle for communication with the ROS system
    ros::NodeHandle nh;

    // Name of the odometry topic to subscribe to
    std::string odometry_topic = "/ze_vio/odometry";  

    // Create a ROS subscriber that listens to the odometry topic and calls the callback function
    ros::Subscriber sub = nh.subscribe(odometry_topic, 1, odometryCallback);
    // queue size = 1, i.e. only the latest message is stored in the queue

    // Name of the output file
    std::string filename="uslam_trajectory.txt";

    // Open file to write odometry data
    outputFile.open(filename.c_str());

    if (!outputFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename.c_str());
        return -1;
    }

    // Keep the node running
    ros::spin();

    // Close the file
    outputFile.close();
    
    return 0;
}

