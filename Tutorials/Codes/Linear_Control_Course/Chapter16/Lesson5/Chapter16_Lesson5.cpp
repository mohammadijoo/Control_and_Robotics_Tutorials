#include <control_toolbox/pid.hpp>
#include <chrono>

class JointVelocityController {
public:
  JointVelocityController(double kp, double ki, double kd, double dt)
    : pid_(kp, ki, kd, -1e6, 1e6), dt_(dt) {}

  double update(double velocity_command, double velocity_measured) {
    double error = velocity_command - velocity_measured;
    // control_toolbox::Pid::computeCommand takes error and dt
    double u = pid_.computeCommand(error, rclcpp::Duration::from_seconds(dt_));
    return u; // send to motor driver
  }

private:
  control_toolbox::Pid pid_;
  double dt_;
};

// In your ROS2 node, choose kp, ki, kd from Bode/Nichols design:
int main() {
  // Example gains obtained by Nichols-chart tuning for desired margins
  double kp = 40.0;
  double ki = 0.0;
  double kd = 0.0;

  double dt = 0.001; // 1 kHz loop
  JointVelocityController ctrl(kp, ki, kd, dt);
  // Inside a timer callback, call ctrl.update(...) each cycle.
}
