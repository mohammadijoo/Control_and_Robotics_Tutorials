
struct JointPID
{
    double Kp{0.0};
    double Kd{0.0};
    double Ki{0.0};

    double integral{0.0};
    double e_prev{0.0};
    double de_f{0.0};   // filtered derivative
    double alpha{0.2};  // derivative filter

    double update(double q, double dq,
                  double qd, double dqd,
                  double dt)
    {
        // tracking error (position only; dqd often 0 for setpoints)
        double e = qd - q;
        double de = (e - e_prev) / dt;
        de_f = (1.0 - alpha) * de_f + alpha * de;

        integral += e * dt;

        // anti-windup (simple clamping)
        const double z_max = 10.0;
        if (integral > z_max) integral = z_max;
        if (integral < -z_max) integral = -z_max;

        double tau = Kp * e + Kd * de_f + Ki * integral;

        e_prev = e;
        return tau;
    }
};

// Example usage in a high-rate control loop
// (e.g. inside a ROS control plugin)
void controlLoopStep(double dt)
{
    static JointPID pid;
    // gains could be parameterized or tuned online
    pid.Kp = 10.0;
    pid.Kd = 1.5;
    pid.Ki = 2.0;

    double q   = readJointPosition();
    double dq  = readJointVelocity();
    double qd  = desiredJointPosition();
    double dqd = 0.0; // for a constant setpoint

    double tau = pid.update(q, dq, qd, dqd, dt);
    sendJointTorqueCommand(tau);
}
