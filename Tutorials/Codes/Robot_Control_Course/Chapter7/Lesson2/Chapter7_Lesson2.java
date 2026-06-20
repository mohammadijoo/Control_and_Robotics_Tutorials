
public class RobustSingleJointPD {

    static class TrajectorySample {
        public final double t;
        public final double e;
        public final double edot;

        public TrajectorySample(double t, double e, double edot) {
            this.t = t;
            this.e = e;
            this.edot = edot;
        }
    }

    public static TrajectorySample[] simulate(double J,
                                             double b,
                                             double k_p,
                                             double k_d,
                                             double e0,
                                             double edot0,
                                             double dt,
                                             double T) {
        int nSteps = (int) Math.round(T / dt);
        TrajectorySample[] traj = new TrajectorySample[nSteps];
        double e = e0;
        double edot = edot0;
        for (int k = 0; k < nSteps; ++k) {
            double tau = -k_p * e - k_d * edot;
            double eddot = (tau - b * edot) / J;
            edot = edot + dt * eddot;
            e = e + dt * edot;
            double t = k * dt;
            traj[k] = new TrajectorySample(t, e, edot);
        }
        return traj;
    }

    public static void main(String[] args) {
        double b = 0.2;
        double k_p = 25.0;
        double k_d = 10.0;

        double[] J_values = {0.5, 1.0, 1.5};
        for (double J : J_values) {
            TrajectorySample[] traj =
                simulate(J, b, k_p, k_d,
                         0.5, 0.0, 1e-3, 2.0);
            TrajectorySample last = traj[traj.length - 1];
            System.out.println("J = " + J
                + ", final e = " + last.e
                + ", final edot = " + last.edot);
        }
    }
}
