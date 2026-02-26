public class SecondOrderSimulation {

    // Standard second-order parameters
    private final double wn;    // natural frequency
    private final double zeta;  // damping ratio
    private final double K;     // DC gain

    private double y;           // output position
    private double ydot;        // output velocity

    public SecondOrderSimulation(double wn, double zeta, double K) {
        this.wn = wn;
        this.zeta = zeta;
        this.K = K;
        this.y = 0.0;
        this.ydot = 0.0;
    }

    // Single simulation step with explicit Euler
    public void step(double u, double dt) {
        double yddot =
            -2.0 * zeta * wn * ydot
            - wn * wn * y
            + K * wn * wn * u;

        ydot += dt * yddot;
        y    += dt * ydot;
    }

    public double getY() {
        return y;
    }

    public static void main(String[] args) {
        SecondOrderSimulation sim =
            new SecondOrderSimulation(4.0, 0.7, 1.0);

        double dt = 0.001;
        double u = 1.0; // step input

        for (int k = 0; k < 5000; ++k) {
            sim.step(u, dt);
            if (k % 1000 == 0) {
                double t = k * dt;
                System.out.println(t + " " + sim.getY());
            }
        }
    }
}
