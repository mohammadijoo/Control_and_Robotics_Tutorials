#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class AddServer : public rclcpp::Node {
public:
  AddServer() : Node("add_server") {
    srv_ = create_service<example_interfaces::srv::AddTwoInts>(
      "/add",
      [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res) {
        res->sum = req->a + req->b;
      }
    );
  }
private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};
      
