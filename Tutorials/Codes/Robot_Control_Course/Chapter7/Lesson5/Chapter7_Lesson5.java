
public class RobustTrackingLab1DOF {

    static final double m = 1.0;
    static final double l = 0.5;
    static final double g = 9.81;
    static final double I = m * l * l;

    static final double lam = 4.0;
    static final double k_s = 5.0;
    static final double phi = 0.1;
    static final double alpha = 30.0;

    static double q_d(double t) {
        return 0.5 * Math.sin(t);
    }

    static double dq_d(double t) {
        return 0.5 * Math.cos(t);
    }

    static double ddq_d(double t) {
        return -0.5 * Math.sin(t);
    }

    static double disturbance(double t, double q, double dq) {
        double sign_dq = 0.0;
        if (dq > 0.0) sign_dq = 1.0;
        if (dq < 0.0) sign_dq = -1.0;
        return 1.5 * Math.sin(3.0 * t) + 0.5 * sign_dq;
    }

    static double sat(double s, double phi) {
        double sigma = s / phi;
        if (sigma > 1.0) return 1.0;
        if (sigma < -1.0) return -1.0;
        return sigma;
    }

    public static void main(String[] args) {
        double t = 0.0;
        double tf = 20.0;
        double dt = 0.001;

        double q = 0.0;
        double dq = 0.0;
        double d_hat = 0.0;

        while (t < tf) {
            double e = q - q_d(t);
            double de = dq - dq_d(t);
            double s = de + lam * e;

            double g_term = m * g * l * Math.sin(q);

            double tau_nom = I * (ddq_d(t) - lam * de) + g_term;
            double tau_rob = -k_s * sat(s, phi);
            double tau = tau_nom + tau_rob - d_hat;

            double d = disturbance(t, q, dq);
            double ddq = (tau + d - g_term) / I;

            double q_ddot_est = ddq;
            double d_hat_dot = -alpha * d_hat + alpha * (I * q_ddot_est + g_term - tau);

            // Euler integration
            q += dt * dq;
            dq += dt * ddq;
            d_hat += dt * d_hat_dot;

            t += dt;

            if (Math.abs((t % 0.5) - 0.25) < dt) {
                System.out.println(t + " " + q + " " + e);
            }
        }
    }
}
