
public class AdaptiveJointLab {

    static class Params {
        double theta1 = 2.0;
        double theta2 = 0.4;
        double theta3 = 5.0;
    }

    static class State {
        double q = 0.0;
        double dq = 0.0;
        double th1_hat = 1.5;
        double th2_hat = 0.2;
        double th3_hat = 4.0;
    }

    static double lam = 5.0;
    static double k_s = 10.0;
    static double g1 = 5.0, g2 = 5.0, g3 = 5.0;
    static double A = 0.5, omega = 1.0;

    static double[] desired(double t) {
        double qd = A * Math.sin(omega * t);
        double dqd = A * omega * Math.cos(omega * t);
        double ddqd = -A * omega * omega * Math.sin(omega * t);
        return new double[]{qd, dqd, ddqd};
    }

    static double[] regressor(double q, double dq,
                              double qd, double dqd, double ddqd) {
        double e = q - qd;
        double de = dq - dqd;
        double dq_r = dqd - lam * e;
        double ddq_r = ddqd - lam * de;
        double Y1 = ddq_r;
        double Y2 = dq_r;
        double Y3 = Math.sin(q);
        double s = dq - dq_r;
        return new double[]{Y1, Y2, Y3, s};
    }

    static double accel(Params p, double q, double dq, double tau) {
        return (tau - p.theta2 * dq - p.theta3 * Math.sin(q)) / p.theta1;
    }

    public static void main(String[] args) {
        Params plant = new Params();
        State x = new State();

        double dt = 0.001;
        double T = 20.0;

        for (double t = 0.0; t <= T; t += dt) {
            double[] dref = desired(t);
            double qd = dref[0], dqd = dref[1], ddqd = dref[2];

            double[] reg = regressor(x.q, x.dq, qd, dqd, ddqd);
            double Y1 = reg[0], Y2 = reg[1], Y3 = reg[2], s = reg[3];

            double tau = Y1 * x.th1_hat + Y2 * x.th2_hat + Y3 * x.th3_hat - k_s * s;
            double ddq = accel(plant, x.q, x.dq, tau);

            x.q += dt * x.dq;
            x.dq += dt * ddq;

            x.th1_hat += dt * (-g1 * Y1 * s);
            x.th2_hat += dt * (-g2 * Y2 * s);
            x.th3_hat += dt * (-g3 * Y3 * s);

            if (((int) (t / dt)) % 1000 == 0) {
                System.out.println(t + " " + x.q);
            }
        }
    }
}
