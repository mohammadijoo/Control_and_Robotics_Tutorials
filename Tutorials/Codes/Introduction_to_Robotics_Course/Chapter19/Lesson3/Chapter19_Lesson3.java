public class JointPDController {
    private final double Kp;
    private final double Kd;
    private final double uMin, uMax;
    private final double dMin, dMax;
    private final int    nCounts;

    public JointPDController(double Kp, double Kd,
                             double uMin, double uMax,
                             double dMin, double dMax,
                             int nCounts) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.uMin = uMin;
        this.uMax = uMax;
        this.dMin = dMin;
        this.dMax = dMax;
        this.nCounts = nCounts;
    }

    private double countsToRad(int counts) {
        return 2.0 * Math.PI * ((double) counts) / ((double) nCounts);
    }

    private double torqueToDuty(double u) {
        double uu = Math.max(uMin, Math.min(uMax, u));
        return (uu - uMin) * (dMax - dMin) / (uMax - uMin) + dMin;
    }

    public double step(int counts, double dq,
                       double qRef, double dqRef) {
        double q  = countsToRad(counts);
        double e  = qRef  - q;
        double de = dqRef - dq;
        double u  = Kp * e + Kd * de;
        return torqueToDuty(u);
    }
}
      
