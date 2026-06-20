
#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct JointTrajectorySample {
    Eigen::VectorXd q_d;   // desired positions
    Eigen::VectorXd q;     // measured positions
};

double computeISE(const std::vector<JointTrajectorySample>& traj,
                  double dt)
{
    if (traj.empty()) return 0.0;
    const int n = traj.front().q_d.size();
    double ise = 0.0;

    for (const auto& s : traj) {
        Eigen::VectorXd e = s.q_d - s.q;
        ise += e.squaredNorm() * dt;
    }
    return ise;
}

int main() {
    const int n = 2;
    double dt = 0.001;
    std::vector<JointTrajectorySample> traj;

    // Fill with some fake data (for illustration)
    for (int k = 0; k < 2000; ++k) {
        double t = k * dt;
        JointTrajectorySample s;
        s.q_d = Eigen::VectorXd(n);
        s.q = Eigen::VectorXd(n);

        s.q_d(0) = 0.5 * std::sin(2.0 * M_PI * t);
        s.q_d(1) = 0.3 * std::sin(4.0 * M_PI * t);

        s.q(0) = 0.48 * std::sin(2.0 * M_PI * (t - 0.01));
        s.q(1) = 0.29 * std::sin(4.0 * M_PI * (t - 0.01));

        traj.push_back(s);
    }

    double ise = computeISE(traj, dt);
    std::cout << "ISE = " << ise << std::endl;
    return 0;
}
