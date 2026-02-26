/*
Chapter 14 - Lesson 5: Lab: Assemble a Full Navigation Stack

C++ ROS 2 example: send a NavigateToPose action goal to Nav2.

Build notes (typical):
- Put this into a ROS 2 package (ament_cmake) and link against rclcpp, rclcpp_action,
  geometry_msgs, nav2_msgs.
- Run this node while Nav2 bringup is active.

This is intentionally minimal and focuses on integration plumbing rather than planner internals.
*/

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

static geometry_msgs::msg::PoseStamped make_goal_pose(
    rclcpp::Node* node, double x, double y, double yaw_rad, const std::string& frame_id = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = node->get_clock()->now();

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
  pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
  return pose;
}

class Chapter14Lesson5NavClient : public rclcpp::Node
{
public:
  Chapter14Lesson5NavClient() : Node("chapter14_lesson5_nav_client")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Chapter14Lesson5NavClient::tick, this));
  }

private:
  void tick()
  {
    if (sent_) return;

    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /navigate_to_pose action server...");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = make_goal_pose(this, 2.0, 0.5, 0.0, "map");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (feedback && std::isfinite(feedback->distance_remaining)) {
          RCLCPP_INFO(this->get_logger(), "Distance remaining: %.3f m", feedback->distance_remaining);
        }
      };
    send_goal_options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Goal reached (SUCCEEDED).");
        } else {
          RCLCPP_WARN(this->get_logger(), "Navigation finished with code: %d", static_cast<int>(result.code));
        }
        rclcpp::shutdown();
      };

    RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose goal...");
    client_->async_send_goal(goal, send_goal_options);
    sent_ = true;
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool sent_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Chapter14Lesson5NavClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
