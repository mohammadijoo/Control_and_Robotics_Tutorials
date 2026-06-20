public class AssistiveJointController {
    private final double Kp;
    private final double Kd;
    private final double tauMax;

    public AssistiveJointController(double J, double B,
                                    double omegaN, double zeta,
                                    double tauMax) {
        this.Kp = J * omegaN * omegaN;
        this.Kd = 2.0 * zeta * omegaN * J - B;
        this.tauMax = tauMax;
    }

    private double saturate(double x) {
        if (x > tauMax) return tauMax;
        if (x < -tauMax) return -tauMax;
        return x;
    }

    public double computeTorque(double theta, double thetaDot,
                                double thetaD, double thetaDDot) {
        double e = thetaD - theta;
        double eDot = thetaDDot - thetaDot;
        double tauCmd = Kp * e + Kd * eDot;
        return saturate(tauCmd);
    }
}
      
