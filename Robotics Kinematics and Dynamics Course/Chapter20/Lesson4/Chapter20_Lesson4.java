public class DifferentiablePendulum {

    static double[] f(double[] x, double tau, double I,
                      double b, double k) {
        double q = x[0];
        double dq = x[1];
        double ddq = (tau - b * dq - k * q) / I;
        return new double[]{dq, ddq};
    }

    static double[] stepEuler(double[] x, double tau, double I,
                              double b, double k, double dt) {
        double[] dx = f(x, tau, I, b, k);
        return new double[]{
            x[0] + dt * dx[0],
            x[1] + dt * dx[1]
        };
    }

    public static void main(String[] args) {
        double dt = 0.01;
        int N = 100;
        double I = 0.5;
        double b = 0.1;
        double k = 1.0;
        double tau = 1.0;

        double[] x = new double[]{0.0, 0.0};
        double[][] xTraj = new double[N + 1][2];
        xTraj[0] = x.clone();

        // Forward rollout
        for (int kstep = 0; kstep < N; ++kstep) {
            x = stepEuler(x, tau, I, b, k, dt);
            xTraj[kstep + 1] = x.clone();
        }

        // Simple reference trajectory
        double[] qRef = new double[N + 1];
        for (int kstep = 0; kstep <= N; ++kstep) {
            double t = dt * kstep;
            qRef[kstep] = 0.5 * Math.sin(2.0 * Math.PI * t);
        }

        // Forward sensitivity S_k = d x_k / dI (2x1 vector)
        double[][] S = new double[N + 1][2];
        S[0][0] = 0.0;
        S[0][1] = 0.0;

        for (int kstep = 0; kstep < N; ++kstep) {
            double q = xTraj[kstep][0];
            double dq = xTraj[kstep][1];

            // Jacobian of f wrt x
            double df_dq0 = 0.0;
            double df_dq1 = -k / I;
            double df_ddq0 = 1.0;
            double df_ddq1 = -b / I;

            // Jx * S_k
            double JxS0 = df_dq0 * S[kstep][0] + df_ddq0 * S[kstep][1];
            double JxS1 = df_dq1 * S[kstep][0] + df_ddq1 * S[kstep][1];

            // df/dI
            double num = tau - b * dq - k * q;
            double dddq_dI = -num / (I * I);
            double df_dI0 = 0.0;
            double df_dI1 = dddq_dI;

            S[kstep + 1][0] = S[kstep][0] + dt * (JxS0 + df_dI0);
            S[kstep + 1][1] = S[kstep][1] + dt * (JxS1 + df_dI1);
        }

        // Gradient dL/dI
        double dLdI = 0.0;
        for (int kstep = 0; kstep <= N; ++kstep) {
            double q = xTraj[kstep][0];
            double err = q - qRef[kstep];
            dLdI += err * S[kstep][0]; // only q-component
        }

        System.out.println("Gradient dL/dI = " + dLdI);
    }
}
      
