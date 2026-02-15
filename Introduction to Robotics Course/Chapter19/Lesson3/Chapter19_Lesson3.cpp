#include <cmath>
#include <algorithm>

class JointPDController {
public:
    JointPDController(double Kp, double Kd,
                      double u_min, double u_max,
                      double d_min, double d_max,
                      int    N_counts)
        : Kp_(Kp), Kd_(Kd),
          u_min_(u_min), u_max_(u_max),
          d_min_(d_min), d_max_(d_max),
          N_counts_(N_counts) {}

    double step(int counts, double dq,
                double q_ref, double dq_ref)
    {
        double q = countsToRad(counts);
        double e  = q_ref  - q;
        double de = dq_ref - dq;
        double u  = Kp_ * e + Kd_ * de;   // torque [Nm]

        // Saturate
        u = std::max(u_min_, std::min(u_max_, u));

        // Convert to duty cycle
        return torqueToDuty(u);
    }

private:
    double countsToRad(int counts) const {
        return 2.0 * M_PI * static_cast<double>(counts)
               / static_cast<double>(N_counts_);
    }

    double torqueToDuty(double u) const {
        return (u - u_min_) * (d_max_ - d_min_) / (u_max_ - u_min_) + d_min_;
    }

    double Kp_, Kd_;
    double u_min_, u_max_;
    double d_min_, d_max_;
    int    N_counts_;
};
      
