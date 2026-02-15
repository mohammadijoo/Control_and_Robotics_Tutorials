#pragma once
#include <cmath>

namespace friction {

inline double sign_smooth(double v, double eps = 1e-3)
{
    return std::tanh(v / eps);
}

inline double coulomb(double qdot, double Fc, double eps = 1e-3)
{
    return Fc * sign_smooth(qdot, eps);
}

inline double viscous(double qdot, double b)
{
    return b * qdot;
}

inline double stribeck(double qdot,
                       double Fc, double Fs, double vs,
                       double alpha = 1.0, double b = 0.0,
                       double eps = 1e-3)
{
    const double sgn = sign_smooth(qdot, eps);
    const double abs_v = std::fabs(qdot);
    const double phi = std::exp(-std::pow(abs_v / vs, alpha));
    return (Fc + (Fs - Fc) * phi) * sgn + b * qdot;
}

struct JointFrictionParams
{
    double Fc;
    double Fs;
    double vs;
    double alpha;
    double b;
};

inline double dynamics_step(double q, double qdot,
                            double tau_act, double dt,
                            double I,
                            const JointFrictionParams& p)
{
    double tau_f = stribeck(qdot, p.Fc, p.Fs, p.vs, p.alpha, p.b);
    double qddot = (tau_act - tau_f) / I;
    // caller updates q, qdot externally; here just return acceleration
    return qddot;
}

} // namespace friction
      
