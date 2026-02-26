
public class MultiRateJointControl {

    public static void main(String[] args) {
        double T_fast = 0.001;
        int m = 10;
        double tf = 1.0;
        int N_steps = (int)(tf / T_fast);

        // State x = [q, qdot]
        double[] x = new double[] {0.0, 0.0};

        // Discrete double-integrator matrices
        double[][] A_f = new double[][] {
            {1.0, T_fast},
            {0.0, 1.0}
        };
        double[] B_f = new double[] {
            0.5 * T_fast * T_fast,
            T_fast
        };

        double[] K_fast = new double[] {0.0, 5.0};
        double K_slow = 50.0;
        double v_slow = 0.0;

        for (int k = 0; k < N_steps; ++k) {
            double t = k * T_fast;

            // Slow outer loop
            if (k % m == 0) {
                double q_des = (t > 0.1) ? 0.5 : 0.0;
                double e_q = q_des - x[0];
                v_slow = K_slow * e_q;
            }

            // Fast inner loop
            double u = v_slow - (K_fast[0] * x[0] + K_fast[1] * x[1]);

            // x_next = A_f * x + B_f * u
            double q_next =
                A_f[0][0] * x[0] + A_f[0][1] * x[1] + B_f[0] * u;
            double qdot_next =
                A_f[1][0] * x[0] + A_f[1][1] * x[1] + B_f[1] * u;

            x[0] = q_next;
            x[1] = qdot_next;
        }

        System.out.println("Final joint position: " + x[0]);
    }
}
