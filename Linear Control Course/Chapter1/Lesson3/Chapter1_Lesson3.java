public class FirstOrderFeedback {

    public static void main(String[] args) {
        double a = 1.0;
        double b = 1.0;
        double K = 4.0;
        double r0 = 1.0;

        double dt = 0.001;
        double T  = 3.0;
        int N = (int) (T / dt);

        double y = 0.0; // plant output

        for (int k = 0; k <= N; ++k) {
            double t = k * dt;

            // Ideal measurement
            double y_m = y;
            // Error
            double e = r0 - y_m;
            // Proportional controller
            double u = K * e;

            // Plant dynamics: y_dot = -a y + b u
            double y_dot = -a * y + b * u;
            y += dt * y_dot;

            if (k % (N / 10) == 0) {
                System.out.println("t = " + t + ", y = " + y);
            }
        }

        // In robotics libraries such as WPILib for FRC robots,
        // a similar loop is wrapped around a PID controller class.
    }
}
