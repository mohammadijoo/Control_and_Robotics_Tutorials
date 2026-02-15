
public class AdaptiveJoint1DOF {
    static class Params {
        double mTrue = 2.0;
        double bTrue = 0.4;
        double g0True = 5.0;
        double mHat = 1.0;
        double bHat = 0.2;
        double g0Hat = 3.0;
    }

    static double[] desiredTrajectory(double t) {
        double qd = 0.5 * Math.sin(0.5 * t);
        double dqd = 0.25 * Math.cos(0.5 * t);
        double ddqd = -0.125 * Math.sin(0.5 * t);
        return new double[]{qd, dqd, ddqd};
    }

    static double controlLaw(double t, double q, double dq,
                             Params p, double kP, double kD,
                             double[] Y, double[] tauHatOut) {
        double[] traj = desiredTrajectory(t);
        double qd = traj[0], dqd = traj[1], ddqd = traj[2];
        double e = q - qd;
        double de = dq - dqd;
        double ddq_r = ddqd - kD * de - kP * e;
        double dq_r = dqd - kP * e;

        Y[0] = ddq_r;
        Y[1] = dq_r;
        Y[2] = Math.sin(q);
        double tauHat = Y[0] * p.mHat + Y[1] * p.bHat + Y[2] * p.g0Hat;
        tauHatOut[0] = tauHat;
        return tauHat;
    }

    public static void main(String[] args) {
        Params p = new Params();
        double kP = 20.0, kD = 8.0;
        double dt = 0.001, T = 5.0;
        int steps = (int)(T / dt);

        double q = 0.0, dq = 0.0;
        double[] Y = new double[3];
        double[] tauHatOut = new double[1];
        for (int k = 0; k < steps; ++k) {
            double t = k * dt;
            double tau = controlLaw(t, q, dq, p, kP, kD, Y, tauHatOut);
            double tauHat = tauHatOut[0];

            // Simple gradient adaptation
            double error = tau - tauHat;
            double gamma = 0.1;
            p.mHat -= gamma * (-Y[0] * error);
            p.bHat -= gamma * (-Y[1] * error);
            p.g0Hat -= gamma * (-Y[2] * error);

            // Plant integration
            double ddq = (tau - p.bTrue * dq - p.g0True * Math.sin(q)) / p.mTrue;
            dq += ddq * dt;
            q += dq * dt;

            if (k % 1000 == 0) {
                System.out.printf("t=%.3f q=%.3f mHat=%.3f%n", t, q, p.mHat);
            }
        }
    }
}
