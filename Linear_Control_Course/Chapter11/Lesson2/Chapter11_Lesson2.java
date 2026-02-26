public class PiControlDemo {
    public static void main(String[] args) {
        double K = 1.0;
        double tau = 0.5;

        double Kp = 1.1;
        double Ki = 3.0;

        double dt = 0.001;
        int N = 5000;

        double x = 0.0;  // plant state
        double integralE = 0.0;
        double r = 1.0;  // step reference

        double y = 0.0;
        double e = 0.0;
        double u = 0.0;

        for (int k = 0; k < N; k++) {
            double t = k * dt;

            y = x;
            e = r - y;

            integralE += e * dt;
            u = Kp * e + Ki * integralE;

            double xDot = -(1.0 / tau) * x + (K / tau) * u;
            x += xDot * dt;

            if (k % 500 == 0) {
                System.out.printf("t = %.3f, y = %.3f, e = %.3f%n", t, y, e);
            }
        }

        System.out.printf("Final output y(T) = %.4f%n", y);
        System.out.printf("Final error e(T)  = %.4f%n", e);
    }
}
