public class SharedAutonomySim {

    static double humanCommand(double time) {
        if (time < 5.0) {
            return 1.0;
        } else {
            return -1.0;
        }
    }

    public static void main(String[] args) {
        double T = 0.01;
        double T_end = 10.0;
        double k_gain = 1.0;
        double alpha = 0.4;
        double U_max = 2.0;

        int N = (int) (T_end / T);
        double x = 0.0;
        double t = 0.0;

        for (int k = 0; k < N; k++) {
            double u_h = humanCommand(t);
            double u_a = -k_gain * x;
            double u = alpha * u_h + (1.0 - alpha) * u_a;

            // Saturation
            if (u > U_max) u = U_max;
            if (u < -U_max) u = -U_max;

            x = x + T * u;
            t = t + T;
        }

        System.out.println("Final position: " + x);
    }
}
      
