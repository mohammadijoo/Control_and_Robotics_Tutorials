public class FirstOrderMetrics {

    public static class Metrics {
        public double tDelay;
        public double tRise;
        public double tSettle;
    }

    public static Metrics computeMetrics(double tau, double tol) {
        Metrics m = new Metrics();
        m.tDelay = -tau * Math.log(0.5);
        m.tRise = tau * Math.log(9.0);
        m.tSettle = -tau * Math.log(tol);
        return m;
    }

    public static double stepResponse(double K, double tau, double t) {
        return K * (1.0 - Math.exp(-t / tau));
    }

    public static void main(String[] args) {
        double K = 2.0;
        double tau = 0.1;
        Metrics m = computeMetrics(tau, 0.02);
        System.out.println("Delay time t_d = " + m.tDelay);
        System.out.println("Rise time t_r = " + m.tRise);
        System.out.println("Settling time t_s = " + m.tSettle);

        double dt = 0.001;
        double Tfinal = 1.0;
        for (double t = 0.0; t <= Tfinal; t += dt) {
            double y = stepResponse(K, tau, t);
            // In a Java-based robotics framework (e.g., some FRC or custom),
            // y could represent the predicted output for a joint or wheel.
        }
    }
}
