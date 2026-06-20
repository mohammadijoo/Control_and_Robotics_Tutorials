public class MassSpringDamperStateSim {

    public static void main(String[] args) {
        // Physical parameters
        double m = 1.0;
        double c = 0.4;
        double k = 4.0;

        double t0 = 0.0;
        double tf = 10.0;
        double h  = 0.001;
        int N = (int) ((tf - t0) / h);

        // State x = [y; y_dot]
        double x1 = 1.0; // y(0)
        double x2 = 0.0; // y_dot(0)
        double u  = 0.0; // zero-input internal dynamics

        for (int k_step = 0; k_step <= N; ++k_step) {
            double t = t0 + k_step * h;

            if (k_step % 1000 == 0) {
                System.out.println(t + " " + x1 + " " + x2);
            }

            double x1_dot = x2;
            double x2_dot = -(k / m) * x1 - (c / m) * x2 + (1.0 / m) * u;

            x1 += h * x1_dot;
            x2 += h * x2_dot;
        }
    }
}
      
