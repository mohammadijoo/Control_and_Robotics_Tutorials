public class FirstOrderSystem {
    private final double K;
    private final double tau;
    private double y;

    public FirstOrderSystem(double K, double tau) {
        this.K = K;
        this.tau = tau;
        this.y = 0.0;
    }

    // Simulate one step with explicit Euler
    public double step(double u, double dt) {
        double dy_dt = (K * u - y) / tau; // tau dy/dt + y = K u
        y = y + dt * dy_dt;
        return y;
    }

    public static void main(String[] args) {
        FirstOrderSystem sys = new FirstOrderSystem(2.0, 0.5);
        double dt = 0.001;
        double Tfinal = 2.0;
        int N = (int) (Tfinal / dt);

        double t = 0.0;
        for (int k = 0; k <= N; ++k) {
            double u = 1.0; // unit step
            double y = sys.step(u, dt);
            if (k % (N / 10) == 0) {
                System.out.println("t = " + t + " s, y = " + y);
            }
            t += dt;
        }
    }
}
