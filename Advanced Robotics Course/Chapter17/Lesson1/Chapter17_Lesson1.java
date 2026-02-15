public class RopeSimulator {
    private final int N = 10;
    private final double mass = 0.05;
    private final double kSpring = 50.0;
    private final double damping = 0.1;
    private final double g = 9.81;
    private final double dt = 5e-4;

    private final double[][] x = new double[N][2];
    private final double[][] v = new double[N][2];
    private final double[] restLengths = new double[N - 1];

    public RopeSimulator() {
        double rest = 0.05;
        for (int i = 0; i < N; ++i) {
            x[i][0] = rest * i;
            x[i][1] = 0.0;
            v[i][0] = 0.0;
            v[i][1] = 0.0;
        }
        for (int i = 0; i < N - 1; ++i) {
            restLengths[i] = rest;
        }
    }

    private void computeForces(double[][] f) {
        for (int i = 0; i < N; ++i) {
            f[i][0] = 0.0;
            f[i][1] = 0.0;
        }

        // Springs
        for (int i = 0; i < N - 1; ++i) {
            double dx = x[i + 1][0] - x[i][0];
            double dy = x[i + 1][1] - x[i][1];
            double L = Math.sqrt(dx * dx + dy * dy);
            if (L > 1e-6) {
                double dirx = dx / L;
                double diry = dy / L;
                double fs = kSpring * (L - restLengths[i]);
                double fx = fs * dirx;
                double fy = fs * diry;
                f[i][0] += fx;
                f[i][1] += fy;
                f[i + 1][0] -= fx;
                f[i + 1][1] -= fy;
            }
        }

        // Damping and gravity
        for (int i = 0; i < N; ++i) {
            f[i][0] -= damping * v[i][0];
            f[i][1] -= damping * v[i][1];
            f[i][1] -= g * mass;
        }
    }

    public void step(int stepIndex) {
        double[][] f = new double[N][2];
        computeForces(f);

        // Anchor first node
        f[0][0] = 0.0;
        f[0][1] = 0.0;
        v[0][0] = 0.0;
        v[0][1] = 0.0;
        x[0][0] = 0.0;
        x[0][1] = 0.0;

        // Drive last node
        double t = stepIndex * dt;
        x[N - 1][0] = 0.05 * (N - 1);
        x[N - 1][1] = 0.1 * Math.sin(2.0 * Math.PI * t);
        v[N - 1][0] = 0.0;
        v[N - 1][1] = 0.0;

        for (int i = 0; i < N; ++i) {
            double ax = f[i][0] / mass;
            double ay = f[i][1] / mass;
            v[i][0] += dt * ax;
            v[i][1] += dt * ay;
            x[i][0] += dt * v[i][0];
            x[i][1] += dt * v[i][1];
        }
    }

    public static void main(String[] args) {
        RopeSimulator sim = new RopeSimulator();
        int steps = 20000;
        for (int step = 0; step < steps; ++step) {
            sim.step(step);
        }
        System.out.println("Simulation finished.");
    }
}
      
