#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <fstream>

// Global variable to store the file stream
std::ofstream outputFile;

// Callback function to handle received odometry data
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Access and process the odometry data here
    ROS_INFO("Received Odometry Data:\nPosition: x=%.2f, y=%.2f, z=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // Write the odometry data to the file
    outputFile << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << std::endl;
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

    std::string filename="uslam_trajectory.txt";

    // creation of a text file
    //FILE *fp;
    //fp = fopen(filename, "w");

    
    outputFile.open(filename.c_str());

    if (!outputFile.is_open()) {
        ROS_ERROR("Could not open file %s", filename.c_str());
        return -1;
    }


    // write to the text file
    //for (int i = 0; i < 10; i++)
    //    fprintf(filename, "This is the line #%d\n", i + 1);

    // Close the file
     outputFile.close();

    // Keep the node running
    ros::spin();

    return 0;
}





 // Name of the output file
    // std::string output_file = "~/Projects/uslam_ws/src/rpg_ultimate_slam_open/data/uslamtrajectory.txt";

    // // Open file to write odometry data
    // outputFile.open(output_file.c_str(),"w");

    // // Check if the file was opened successfully
    // if (!outputFile.is_open()) {
    //     ROS_ERROR("Could not open file %s", output_file.c_str());
    //     return -1;
    // }

    // // Close the file
    // outputFile.close();

    // // Open file to write odometry data
    // char *filename = "uslam_trajectory.txt";







// // Open file to write odometry data
    // FILE *fp;
    // fp = fopen(filename, "w");
    // // if you give it write permission, it will create the file if it doesn't exist
    // // if you give it read permission, it will return NULL if the file doesn't exist
    // // if the file already exists, it will write over it? no, it will write at the end of the file
    // if (fp == NULL) {
    //     printf("Error opening file %s", filename);
    //     exit(1);
    // }

    // // write to the text file
    // for (int i = 0; i < 10; i++)
    //     fprintf(fp, "This is the line #%d\n", i + 1);

    // // close the file
    // fclose(fp);
