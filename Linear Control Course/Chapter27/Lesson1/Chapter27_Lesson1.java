public class TrackingVsRegulationDemo {

    public static void main(String[] args) {
        double tau = 0.5;
        double Kp  = 4.0;
        double Ki  = 5.0;

        double h   = 0.001;
        double T   = 5.0;
        int N      = (int) (T / h);

        double[] t      = new double[N];
        double[] yTrack = new double[N];
        double[] yReg   = new double[N];

        // Tracking scenario
        double x = 0.0;
        double u = 0.0;
        double ePrev = 0.0;

        for (int k = 0; k < N; ++k) {
            double tk = k * h;
            t[k] = tk;

            double r = 1.0;
            double d = 0.0;

            double y = x + d;
            double e = r - y;

            double du = Kp * (e - ePrev) + Ki * h * e;
            u += du;
            ePrev = e;

            double dx = (-1.0 / tau) * x + (1.0 / tau) * u;
            x += h * dx;

            yTrack[k] = y;
        }

        // Regulation scenario
        x = 0.0;
        u = 0.0;
        ePrev = 0.0;

        for (int k = 0; k < N; ++k) {
            double tk = k * h;

            double r = 0.0;
            double d = (tk >= 1.0) ? 0.5 : 0.0;

            double y = x + d;
            double e = r - y;

            double du = Kp * (e - ePrev) + Ki * h * e;
            u += du;
            ePrev = e;

            double dx = (-1.0 / tau) * x + (1.0 / tau) * u;
            x += h * dx;

            yReg[k] = y;
        }

        // Print a few samples
        for (int k = 0; k < N; k += N / 10) {
            System.out.println(String.format(
                "t = %.3f, yTrack = %.3f, yReg = %.3f",
                t[k], yTrack[k], yReg[k]
            ));
        }
    }
}
