#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dvs_sensor_optical_transform_publisher");
    ros::NodeHandle nh;

    // Create a TransformListener to listen to the dvs_sensor_2_dvs_sensor_optical transform
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Create a TransformBroadcaster to publish the transform
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Create a publisher to publish the transform on a topic
    ros::Publisher transform_pub = nh.advertise<geometry_msgs::TransformStamped>("dvs_sensor_optical_transform", 10);

    // Loop at 10 Hz
    ros::Rate rate(10.0);
    while (ros::ok()) {
        // Get the transform from dvs_sensor to dvs_sensor_optical
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer.lookupTransform("dvs_sensor_optical_frame", "dvs_sensor", ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Publish the transform
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "dvs_sensor_optical_frame";
        transform.child_frame_id = "dvs_sensor";
        tf_broadcaster.sendTransform(transform);

        // Publish the transform using the TransformBroadcaster
        tf_broadcaster.sendTransform(transform);

        // Sleep for the remainder of the loop
        rate.sleep();
    }

    return 0;
}