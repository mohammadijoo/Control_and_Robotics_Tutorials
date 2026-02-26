// Chapter5_Lesson1.cpp
// Wheel Odometry Computation — Differential Drive (planar SE(2))
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter5_Lesson1.cpp -o ch5_l1
//
// Optional: Eigen is used only for demonstration and can be removed.

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0}; // yaw [rad]
};

struct DiffDriveParams {
  double r_l{0.05};          // left wheel radius [m]
  double r_r{0.05};          // right wheel radius [m]
  double b{0.30};            // baseline [m]
  int32_t ticks_per_rev{2048};
  double gear_ratio{1.0};    // motor rev per wheel rev
};

static inline double normalizeAngle(double theta) {
  return std::atan2(std::sin(theta), std::cos(theta)); // (-pi, pi]
}

class DifferentialDriveOdometry {
public:
  explicit DifferentialDriveOdometry(const DiffDriveParams& p, const Pose2D& pose0 = Pose2D())
  : params_(p), pose_(pose0) {}

  double ticksToWheelAngle(int32_t dN) const {
    const double denom = static_cast<double>(params_.ticks_per_rev) * params_.gear_ratio;
    return 2.0 * M_PI * (static_cast<double>(dN) / denom);
  }

  Pose2D updateFromTicks(int32_t dN_L, int32_t dN_R) {
    const double dphi_L = ticksToWheelAngle(dN_L);
    const double dphi_R = ticksToWheelAngle(dN_R);

    const double ds_L = params_.r_l * dphi_L;
    const double ds_R = params_.r_r * dphi_R;

    const double delta_s = 0.5 * (ds_R + ds_L);
    const double delta_theta = (ds_R - ds_L) / params_.b;

    const double eps = 1e-12;

    if (std::abs(delta_theta) < eps) {
      pose_.x += delta_s * std::cos(pose_.theta);
      pose_.y += delta_s * std::sin(pose_.theta);
      pose_.theta = normalizeAngle(pose_.theta + delta_theta);
    } else {
      const double R = delta_s / delta_theta;
      const double th = pose_.theta;
      pose_.x += R * (std::sin(th + delta_theta) - std::sin(th));
      pose_.y += -R * (std::cos(th + delta_theta) - std::cos(th));
      pose_.theta = normalizeAngle(th + delta_theta);
    }

    return pose_;
  }

  Pose2D pose() const { return pose_; }

private:
  DiffDriveParams params_;
  Pose2D pose_;
};

int main() {
  DiffDriveParams p;
  p.r_l = 0.05; p.r_r = 0.05; p.b = 0.30; p.ticks_per_rev = 2048; p.gear_ratio = 1.0;

  DifferentialDriveOdometry odo(p);

  for (int k = 0; k < 200; ++k) {
    odo.updateFromTicks(40, 60);
  }

  const Pose2D out = odo.pose();
  std::cout << "Final pose: x=" << out.x << " y=" << out.y << " theta=" << out.theta << " rad\n";
  return 0;
}
