#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "base_to_laser_broadcaster");
  ros::NodeHandle nh;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped t;

  ros::Rate rate(30.0);
  while (ros::ok()){
    t.header.stamp = ros::Time::now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser_frame";

    // Example fixed offset: laser mounted 0.2 m forward, 0.1 m up
    t.transform.translation.x = 0.2;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.1;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // roll, pitch, yaw
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br.sendTransform(t);
    rate.sleep();
  }
  return 0;
}
      
