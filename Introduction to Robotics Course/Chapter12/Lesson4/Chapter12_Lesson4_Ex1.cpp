#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class BroadcasterNode : public rclcpp::Node {
public:
  BroadcasterNode() : Node("ros2_broadcaster") {
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&BroadcasterNode::tick, this)
    );
  }
private:
  void tick(){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id  = "tool_frame";

    t.transform.translation.x = 0.4;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.2;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br_->sendTransform(t);
  }
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
      
