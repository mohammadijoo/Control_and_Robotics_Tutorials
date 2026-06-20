#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <onnxruntime_cxx_api.h>

class VLAControlNode : public rclcpp::Node {
public:
    VLAControlNode()
    : Node("vla_control_node"),
      env_(ORT_LOGGING_LEVEL_WARNING, "vla"),
      session_(nullptr)
    {
        Ort::SessionOptions opts;
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_ = Ort::Session(env_, "vla_policy.onnx", opts);

        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VLAControlNode::imageCallback, this, std::placeholders::_1));

        state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&VLAControlNode::stateCallback, this, std::placeholders::_1));

        instr_sub_ = create_subscription<std_msgs::msg::String>(
            "/instruction", 10,
            std::bind(&VLAControlNode::instructionCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<trajectory_msgs::msg::Joint_trajectory>(
            "/arm_controller/joint_trajectory", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VLAControlNode::controlLoop, this));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // TODO: convert to float tensor [1, 3, H, W]
        last_image_ = msg;
    }

    void stateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_state_ = msg;
    }

    void instructionCallback(const std_msgs::msg::String::SharedPtr msg) {
        last_instruction_ = msg->data;
        // TODO: tokenize using the same vocabulary as training
    }

    void controlLoop() {
        if (!last_image_ || !last_state_ || last_instruction_.empty()) return;

        // Build ONNX input tensors: image, state, tokens
        std::vector<const char*> input_names = {"image", "state", "tokens"};
        std::vector<Ort::Value> input_tensors;
        // TODO: fill input_tensors with Ort::Value::CreateTensor(...)

        std::vector<const char*> output_names = {"mu", "log_std"};
        auto outputs = session_.Run(
            Ort::RunOptions{nullptr},
            input_names.data(), input_tensors.data(), input_tensors.size(),
            output_names.data(), output_names.size());

        // Extract mu from outputs[0] and construct a JointTrajectory
        trajectory_msgs::msg::Joint_trajectory traj;
        // TODO: map mu to joint velocities or positions
        cmd_pub_->publish(traj);
    }

    Ort::Env env_;
    Ort::Session session_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr instr_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::Joint_trajectory>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Image::SharedPtr last_image_;
    sensor_msgs::msg::JointState::SharedPtr last_state_;
    std::string last_instruction_;
};
      
