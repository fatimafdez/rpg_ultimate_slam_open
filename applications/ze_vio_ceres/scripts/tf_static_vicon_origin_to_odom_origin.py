#!/usr/bin/env python3

# Launch this script to publish a static transform between the vicon origin and the odom origin
# at the beginning of the experiment when base_link ~= odom.

import rospy
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from tf2_ros import StaticTransformBroadcaster

class ViconTFPublisher():
   def __init__(self, node_name):
      rospy.init_node(node_name, anonymous=False)

      vicon_topic = rospy.get_param('~vicon_pose_topic', default="/vicon_client/dvxplorer02/pose")
      self.odom_frame = rospy.get_param('~odom_frame',   default="map") # odom o map
      self.__vicon_sub = rospy.Subscriber(vicon_topic, PoseStamped, self.__vicon_cb)
      self.__tf_pub = StaticTransformBroadcaster()

   def __vicon_cb(self, p):
      tf = TransformStamped()
      tf.header.stamp    = rospy.Time.now()
      tf.header.frame_id = 'vicon'
      tf.child_frame_id  = self.odom_frame
      tf.transform = Transform()
      tf.transform.translation = p.pose.position
      tf.transform.rotation    = p.pose.orientation

      self.__tf_pub.sendTransform(tf)
      print("Publishing static transform between {} and {}-> {}"
         .format(tf.header.frame_id, tf.child_frame_id, tf.transform))
      self.__vicon_sub.unregister()

if __name__ == '__main__':
   vicon_pub = ViconTFPublisher('initial_vicon_tf_publisher')
   rospy.spin()
