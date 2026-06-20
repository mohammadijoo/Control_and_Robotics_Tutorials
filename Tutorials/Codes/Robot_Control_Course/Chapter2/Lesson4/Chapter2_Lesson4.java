
public class JointPDController {
    private double Kp;
    private double Kd;

    public JointPDController(double J, double B, double MpStar, double TsStar) {
        double zeta = dampingRatioFromOvershoot(MpStar);
        double wn   = 4.0 / (zeta * TsStar);
        this.Kp = J * wn * wn;
        this.Kd = 2.0 * J * zeta * wn - B;
        if (this.Kd < 0.0) {
            this.Kd = 0.0;
        }
    }

    public static double dampingRatioFromOvershoot(double MpStar) {
        if (MpStar <= 0.0 || MpStar >= 1.0) {
            throw new IllegalArgumentException("MpStar must be in (0, 1).");
        }
        double logMp = Math.log(MpStar);
        return -logMp / Math.sqrt(Math.PI * Math.PI + logMp * logMp);
    }

    public double computeTorque(double q, double qd, double dq) {
        double e  = q - qd;
        double de = dq; // assume qd constant
        return -Kp * e - Kd * de;
    }

    public double getKp() { return Kp; }
    public double getKd() { return Kd; }

    public static void main(String[] args) {
        double J = 0.5;
        double B = 0.05;
        JointPDController ctrl = new JointPDController(J, B, 0.1, 0.5);

        double tau = ctrl.computeTorque(0.1, 0.0, 0.0);
        System.out.println("Kp = " + ctrl.getKp() + ", Kd = " + ctrl.getKd());
        System.out.println("tau = " + tau);
    }
}
