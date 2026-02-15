public class CubicTrajectory {
    private final double qFinal;
    private final double T;

    public CubicTrajectory(double qFinal, double Tmove) {
        this.qFinal = qFinal;
        this.T = Tmove;
    }

    public double position(double t) {
        if (t <= 0.0) return 0.0;
        if (t >= T)   return qFinal;
        double s = t / T;
        return 3.0 * qFinal * s * s - 2.0 * qFinal * s * s * s;
    }

    public double velocity(double t) {
        if (t <= 0.0 || t >= T) return 0.0;
        return (6.0 * qFinal / (T * T)) * t
             - (6.0 * qFinal / (T * T * T)) * t * t;
    }

    // Example integration with a Java-based control loop:
    // CubicTrajectory traj = new CubicTrajectory(1.0, 1.0);
    // double t = 0.0;
    // double dt = 0.001;
    // while (robotIsEnabled()) {
    //     double qRef = traj.position(t);
    //     // send qRef to a PID controller for the joint or axis
    //     t += dt;
    // }
}
