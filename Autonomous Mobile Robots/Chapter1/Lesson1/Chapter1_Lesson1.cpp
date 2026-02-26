// Chapter1_Lesson1.cpp
// Autonomous Mobile Robots (Control Engineering) — Chapter 1, Lesson 1
// Lesson: What Makes Mobility Different from Manipulation
//
// Minimal C++17 simulation that contrasts:
//   (i) differential-drive mobile base planar kinematics
//   (ii) planar 2R manipulator kinematics
//
// Robotics libraries commonly used in C++ ecosystems (not required here):
//   - Eigen (linear algebra)
//   - ROS 2 (rclcpp) for robotics middleware
//   - GTSAM / Ceres for estimation (later chapters)
//
// Build:
//   g++ -std=c++17 -O2 -o Chapter1_Lesson1 Chapter1_Lesson1.cpp
//
// Run:
//   ./Chapter1_Lesson1
//
// Output:
//   Chapter1_Lesson1_mobile.csv
//   Chapter1_Lesson1_manipulator.csv

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct Pose2 {
    double x;
    double y;
    double theta;
};

static std::vector<Pose2> integrate_diff_drive(const Pose2& p0, double v, double w, double dt, int steps) {
    std::vector<Pose2> traj;
    traj.reserve(static_cast<size_t>(steps) + 1);

    Pose2 p = p0;
    traj.push_back(p);

    for (int k = 0; k < steps; ++k) {
        p.x += dt * v * std::cos(p.theta);
        p.y += dt * v * std::sin(p.theta);
        p.theta += dt * w;
        traj.push_back(p);
    }
    return traj;
}

static void fk_2r(double q1, double q2, double l1, double l2, double& xe, double& ye) {
    xe = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    ye = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
}

int main() {
    const double dt = 0.02;
    const double T  = 8.0;
    const int steps = static_cast<int>(T / dt);

    // Mobile base: body-frame controls v, w
    const double v = 0.5;
    const double w = 0.35;

    auto mobile_traj = integrate_diff_drive(Pose2{0.0, 0.0, 0.0}, v, w, dt, steps);

    {
        std::ofstream f("Chapter1_Lesson1_mobile.csv");
        f << "t,x,y,theta\n";
        for (int k = 0; k < static_cast<int>(mobile_traj.size()); ++k) {
            const auto& p = mobile_traj[static_cast<size_t>(k)];
            f << (k * dt) << "," << p.x << "," << p.y << "," << p.theta << "\n";
        }
    }

    // 2R manipulator: joint-rate controls qdot
    const double qdot1 = 0.25;
    const double qdot2 = -0.15;
    double q1 = 0.2;
    double q2 = 0.9;
    const double l1 = 1.0;
    const double l2 = 0.7;

    {
        std::ofstream f("Chapter1_Lesson1_manipulator.csv");
        f << "t,q1,q2,xe,ye\n";
        for (int k = 0; k <= steps; ++k) {
            double xe = 0.0, ye = 0.0;
            fk_2r(q1, q2, l1, l2, xe, ye);
            f << (k * dt) << "," << q1 << "," << q2 << "," << xe << "," << ye << "\n";

            q1 += dt * qdot1;
            q2 += dt * qdot2;
        }
    }

    std::cout << "Wrote CSV files for mobile base and 2R manipulator trajectories." << std::endl;
    return 0;
}
