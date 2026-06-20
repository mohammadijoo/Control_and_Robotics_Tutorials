
#include <cmath>
#include <algorithm>

struct JointServo
{
    double Kp;
    double Kd;
    double Ki;
    double tau_min;
    double tau_max;

    double integral;  // integral state

    JointServo(double kp, double kd, double ki,
               double tmin, double tmax)
        : Kp(kp), Kd(kd), Ki(ki),
          tau_min(tmin), tau_max(tmax),
          integral(0.0)
    {}

    double gravity_torque(double q)
    {
        // Example: planar link with m g l sin(q)
        const double m = 1.0;
        const double ell = 0.5;
        const double g = 9.81;
        return m * g * ell * std::sin(q);
    }

    double update(double q, double dq,
                  double qd, double dqd,
                  double dt)
    {
        double e = qd - q;
        double de = dqd - dq;

        // integrate error with simple clamping
        integral += e * dt;
        const double Imax = 5.0;
        if (integral > Imax) integral = Imax;
        if (integral < -Imax) integral = -Imax;

        double tau = Kp * e + Kd * de + Ki * integral
                     + gravity_torque(q);

        // saturation
        tau = std::clamp(tau, tau_min, tau_max);
        return tau;
    }
};

// In your real-time loop:
//   - dt is the measured control period
//   - q, dq from encoders/velocity estimator
//   - qd, dqd from trajectory generator
// JointServo servo(Kp, Kd, Ki, -10.0, 10.0);
// double tau_cmd = servo.update(q, dq, qd, dqd, dt);
