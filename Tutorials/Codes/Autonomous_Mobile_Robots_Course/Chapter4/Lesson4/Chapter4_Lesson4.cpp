// Chapter4_Lesson4.cpp
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 4: Mobile Robot Dynamics (Applied)
Lesson 4: Stability and Tip-Over Risk (vehicle view)

C++ implementation of quasi-static tip-over and sliding checks.

Optional robotics stack integration:
- Eigen for vector math
- ROS 2 (rclcpp) for subscribing to planned trajectories or odometry

This file is self-contained and compiles without ROS when Eigen is available.
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>

struct VehicleParams {
    double track;      // [m]
    double wheelbase;  // [m]
    double cgHeight;   // [m]
    double mu;         // [-]
    double g;          // [m/s^2]
};

static inline double tipThresholdLateral(const VehicleParams& p) {
    return p.g * (p.track / (2.0 * p.cgHeight));
}

static inline double tipThresholdLongitudinal(const VehicleParams& p) {
    return p.g * (p.wheelbase / (2.0 * p.cgHeight));
}

static inline double slideThreshold(const VehicleParams& p) {
    return p.mu * p.g;
}

static inline double ltrLateral(const VehicleParams& p, double aY) {
    // LTR_y = 2*h*a_y/(g*T)
    return 2.0 * p.cgHeight * aY / (p.g * p.track);
}

static inline double ltrLongitudinal(const VehicleParams& p, double aX) {
    // LTR_x = 2*h*a_x/(g*L)
    return 2.0 * p.cgHeight * aX / (p.g * p.wheelbase);
}

static inline double lateralAccel(double v, double kappa) {
    return v * v * kappa;
}

int main() {
    VehicleParams p{0.55, 0.65, 0.25, 0.70, 9.81};

    std::cout << "=== Vehicle params ===\n";
    std::cout << "track=" << p.track << " wheelbase=" << p.wheelbase
              << " cgHeight=" << p.cgHeight << " mu=" << p.mu << "\n";
    std::cout << "a_tip_y=" << tipThresholdLateral(p) << " [m/s^2]\n";
    std::cout << "mu*g=" << slideThreshold(p) << " [m/s^2]\n\n";

    // Synthetic samples: (t, v, kappa)
    const int N = 401;
    const double t0 = 0.0, t1 = 10.0;
    std::vector<double> t(N), v(N), kappa(N);
    for (int i = 0; i < N; ++i) {
        double ti = t0 + (t1 - t0) * (static_cast<double>(i) / (N - 1));
        t[i] = ti;

        // Speed profile
        double vi = 0.2 + 1.5 * (1.0 - std::exp(-ti / 2.2));
        if (ti > 7.0) vi *= (1.0 - 0.35 * (ti - 7.0) / 3.0);
        v[i] = std::clamp(vi, 0.0, 1.9);

        // Curvature profile
        double ki = 0.05 + 0.35 * std::exp(-std::pow((ti - 5.0) / 1.8, 2.0));
        kappa[i] = ki;
    }

    // Compute metrics
    double minMarginTipY = 1e9;
    double minMarginSlide = 1e9;

    for (int i = 1; i < N; ++i) {
        double dt = t[i] - t[i - 1];
        double aX = (v[i] - v[i - 1]) / dt;  // finite difference
        double aY = lateralAccel(v[i], kappa[i]);

        double LTRy = ltrLateral(p, aY);
        double marginTipY = 1.0 - std::abs(LTRy);

        double aPlanar = std::sqrt(aX * aX + aY * aY);
        double marginSlide = slideThreshold(p) - aPlanar;

        minMarginTipY = std::min(minMarginTipY, marginTipY);
        minMarginSlide = std::min(minMarginSlide, marginSlide);
    }

    std::cout << "Min margin_tip_y = " << minMarginTipY << "\n";
    std::cout << "Min margin_slide = " << minMarginSlide << "\n";

    // Robotics note (ROS 2 idea):
    // - subscribe to nav_msgs::Path to obtain curvature estimates
    // - subscribe to geometry_msgs::TwistStamped for current speed
    // - publish a safety-limited speed command if margins become small

    return 0;
}
