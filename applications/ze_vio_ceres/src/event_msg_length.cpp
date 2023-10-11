#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

void messageCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
    int numEvents = msg->events.size();
    ROS_INFO("Received %d events on /dvs/events.", numEvents);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/dvs/events", 10, messageCallback);

    ros::spin();

    return 0;
}