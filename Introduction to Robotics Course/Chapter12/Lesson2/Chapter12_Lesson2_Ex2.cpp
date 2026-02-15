#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

class FibServer : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  FibServer() : Node("fib_server") {
    server_ = rclcpp_action::create_server<Fibonacci>(
      this, "/fib",
      std::bind(&FibServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FibServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FibServer::handle_accept, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Fibonacci::Goal> goal) {
    return goal->order > 0 ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                           : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandle> gh) {
    std::thread{std::bind(&FibServer::execute, this, gh)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    feedback->sequence = {0,1};

    for(int i=2; i<gh->get_goal()->order; ++i){
      feedback->sequence.push_back(feedback->sequence[i-1]+feedback->sequence[i-2]);
      gh->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    result->sequence = feedback->sequence;
    gh->succeed(result);
  }
};
      
