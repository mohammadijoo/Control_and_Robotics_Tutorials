#include <Eigen/Dense>
#include <cmath>

class FirstOrderLPF {
public:
  FirstOrderLPF(double wf, double Ts)
    : alpha_(std::exp(-wf * Ts)), y_f_(0.0) {}

  double update(double y_meas) {
    y_f_ = alpha_ * y_f_ + (1.0 - alpha_) * y_meas;
    return y_f_;
  }

private:
  double alpha_;
  double y_f_;
};

class JointPDController {
public:
  JointPDController(double Kp, double Kd, double wf, double Ts)
    : Kp_(Kp), Kd_(Kd), lpf_(wf, Ts), prev_pos_(0.0) {}

  double compute(double r, double y_meas, double Ts) {
    // Measurement-path filter: filter the measured position
    double y_f = lpf_.update(y_meas);

    // PD on filtered measurement
    double e = r - y_f;
    double vel_est = (y_f - prev_pos_) / Ts;
    prev_pos_ = y_f;

    double u = Kp_ * e - Kd_ * vel_est;
    return u;
  }

private:
  double Kp_, Kd_;
  FirstOrderLPF lpf_;
  double prev_pos_;
};
