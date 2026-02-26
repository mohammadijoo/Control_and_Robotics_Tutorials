#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions

rospy.init_node("base_to_camera_broadcaster")
br = tf2_ros.TransformBroadcaster()
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "camera_frame"

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.15
    t.transform.translation.z = 0.25

    q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 1.57)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)
    rate.sleep()
      
