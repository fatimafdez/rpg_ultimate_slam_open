#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::Path path;
ros::Publisher path_pub;

void viconPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    // Create a PoseStamped message with the pose
    geometry_msgs::PoseStamped pose_stamped = *pose;
    
    // Set the frame ID (frame in which Vicon poses are published)
    pose_stamped.header.frame_id = "vicon";

    // Set the timestamp (use the time when you received the Vicon pose)
    pose_stamped.header.stamp = ros::Time::now();

    // Add the pose_stamped to the path
    path.poses.push_back(pose_stamped);

    // Publish the path
    path_pub.publish(path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_trajectory_publisher");
    ros::NodeHandle nh;

    bool use_vicon;
    nh.param("use_vicon", use_vicon, false);

    if (use_vicon)
    {
        // Initialize the path and frame ID
        path.header.frame_id = "vicon";

        // Create a publisher for the path
        path_pub = nh.advertise<nav_msgs::Path>("vicon_path", 10);

        // Subscribe to the Vicon pose topic
        std::string vicon_topic_name;
        nh.getParam("vicon_topic_name", vicon_topic_name);
        ros::Subscriber vicon_sub = nh.subscribe<geometry_msgs::PoseStamped>(vicon_topic_name, 10, viconPoseCallback);

        ros::spin();
    }
    
    return 0;
}