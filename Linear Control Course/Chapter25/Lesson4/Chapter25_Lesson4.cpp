#include <ros/ros.h>
#include <control_toolbox/pid.h>

class JointController {
public:
  JointController()
  : pid_(/*kp=*/8.0, /*ki=*/4.0, /*kd=*/0.0, /*i_max=*/10.0, /*i_min=*/-10.0)
  {}

  double update(double ref_pos,
                double measured_pos,
                double measured_disturbance,
                double dt)
  {
    // Feedback term on position error
    double error = ref_pos - measured_pos;
    double u_fb = pid_.computeCommand(error, ros::Duration(dt));

    // Disturbance feedforward: F(s) = -1
    double u_ff = -measured_disturbance;

    // Total control input (e.g., desired torque or current)
    double u = u_fb + u_ff;
    return u;
  }

private:
  control_toolbox::Pid pid_;
};
