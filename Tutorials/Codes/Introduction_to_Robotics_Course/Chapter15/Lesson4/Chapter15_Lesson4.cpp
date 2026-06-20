#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor()
    : Node("safety_monitor"), speed_(0.0), distance_(1e9)
    {
        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/robot/speed", 10,
            std::bind(&SafetyMonitor::speedCallback, this, _1));

        dist_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/human/distance", 10,
            std::bind(&SafetyMonitor::distanceCallback, this, _1));

        estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/safety/estop", 10);

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(
            10ms, std::bind(&SafetyMonitor::checkSafety, this));
    }

private:
    void speedCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        speed_ = msg->data;
    }

    void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        distance_ = msg->data;
    }

    void checkSafety() {
        const double SAFE_DISTANCE = 0.8;
        const double SPEED_LIMIT  = 0.25;
        if (distance_ <= SAFE_DISTANCE && speed_ > SPEED_LIMIT) {
            RCLCPP_WARN(this->get_logger(), "Safety violation, triggering E-STOP");
            std_msgs::msg::Bool msg;
            msg.data = true;
            estop_pub_->publish(msg);
        }
    }

    double speed_;
    double distance_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}
      
