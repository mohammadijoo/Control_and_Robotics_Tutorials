
public class AdaptiveCT1DOF {

    private double lambda = 10.0;
    private double kD = 5.0;

    // theta_hat = [theta1, theta2, theta3]^T
    private double[] thetaHat = {0.5, 0.1, 0.5};
    // Gamma is diagonal
    private double[] gammaDiag = {5.0, 1.0, 1.0};

    public double step(double q, double qd,
                       double q_d, double qd_d, double qdd_d,
                       double dt) {

        double tilde_q  = q - q_d;
        double tilde_qd = qd - qd_d;

        double qrd  = qd_d - lambda * tilde_q;
        double qrdd = qdd_d - lambda * tilde_qd;
        double s    = qd - qrd;

        double[] Y = new double[3];
        Y[0] = qrdd;
        Y[1] = qrd;
        Y[2] = Math.cos(q);

        double tau = Y[0] * thetaHat[0]
                   + Y[1] * thetaHat[1]
                   + Y[2] * thetaHat[2]
                   + kD * s;

        // theta_dot = -Gamma * Y^T * s
        double[] thetaDot = new double[3];
        for (int i = 0; i < 3; ++i) {
            thetaDot[i] = -gammaDiag[i] * Y[i] * s;
            thetaHat[i] += thetaDot[i] * dt;
        }

        return tau;
    }

    public double[] getThetaHat() {
        return thetaHat.clone();
    }
}
