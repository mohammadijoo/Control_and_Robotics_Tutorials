#include <ros/ros.h>
#include <std_msgs/Float64.h>

class TwoDofPID {
public:
  TwoDofPID(double Kp, double Ki, double Kd,
            double beta, double gamma, double Ts)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      beta_(beta), gamma_(gamma), Ts_(Ts),
      e_int_(0.0),
      r_prev_(0.0), r_prev2_(0.0),
      y_prev_(0.0), y_prev2_(0.0) {}

  double update(double r, double y) {
    // Tracking error
    double e = r - y;

    // Integral term (simple rectangular integration)
    e_int_ += e * Ts_;

    // "Weighted" signals for proportional and derivative actions
    double r_beta = beta_ * r;
    double r_gamma = gamma_ * r;

    // Discrete derivative approximation (second-order backward difference)
    double dr = (r_gamma - 2.0 * r_prev_ + r_prev2_) / (Ts_ * Ts_);
    double dy = (y - 2.0 * y_prev_ + y_prev2_) / (Ts_ * Ts_);

    double u =
      Kp_ * (r_beta - y) +        // proportional
      Ki_ * e_int_ +             // integral
      Kd_ * (dr - dy);           // derivative on weighted reference and output

    // Shift histories
    r_prev2_ = r_prev_;
    r_prev_  = r_gamma;
    y_prev2_ = y_prev_;
    y_prev_  = y;

    return u;
  }

private:
  double Kp_, Ki_, Kd_;
  double beta_, gamma_;
  double Ts_;
  double e_int_;
  double r_prev_, r_prev2_;
  double y_prev_, y_prev2_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_2dof_pid");
  ros::NodeHandle nh;

  // Example parameters (would be loaded from ROS params)
  double Kp = 44.4;
  double Ki = 10.0;
  double Kd = 8.0;
  double beta = 0.5;
  double gamma = 0.0;
  double Ts = 0.001; // 1 ms

  TwoDofPID pid(Kp, Ki, Kd, beta, gamma, Ts);

  ros::Publisher cmd_pub =
    nh.advertise<std_msgs::Float64>("/joint1_effort_controller/command", 1);

  ros::Rate rate(1.0 / Ts);

  double r = 1.0;   // desired joint angle [rad]

  while (ros::ok()) {
    // Here we would read the measured joint position y from a topic or interface
    double y_meas = /* read from hardware or simulation */ 0.0;

    double u = pid.update(r, y_meas);

    std_msgs::Float64 msg;
    msg.data = u;
    cmd_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
