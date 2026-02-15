
public class JointDOB {
    private final double Jnom;
    private final double wc;
    private final double dt;

    private double tauUHat = 0.0;
    private double qPrev = 0.0;
    private double qdotPrev = 0.0;
    private boolean initialized = false;

    public JointDOB(double Jnom, double wc, double dt) {
        this.Jnom = Jnom;
        this.wc = wc;
        this.dt = dt;
    }

    public double update(double q, double qdot, double tauC) {
        if (!initialized) {
            qPrev = q;
            qdotPrev = qdot;
            initialized = true;
        }

        double qddot = (qdot - qdotPrev) / dt;

        double rhs = -wc * tauUHat + wc * (Jnom * qddot - tauC);
        tauUHat = tauUHat + dt * rhs;

        double tau = tauC - tauUHat;

        qPrev = q;
        qdotPrev = qdot;
        return tau;
    }
}

// Usage:
// JointDOB dob = new JointDOB(1.0, 50.0, 0.001);
// double tau = dob.update(q, qdot, tauC);
