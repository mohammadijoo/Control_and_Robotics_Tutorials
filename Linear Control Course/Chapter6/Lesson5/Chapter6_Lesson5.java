public class DominantSecondOrder {
    private double wn;
    private double zeta;
    private double k;
    private double x1;  // output y
    private double x2;  // derivative y_dot

    public DominantSecondOrder(double wn, double zeta, double k) {
        this.wn = wn;
        this.zeta = zeta;
        this.k = k;
        this.x1 = 0.0;
        this.x2 = 0.0;
    }

    // Compute derivatives for unit-step input u(t) = 1
    public void derivatives(double[] dx) {
        double u = 1.0;
        dx[0] = x2;
        dx[1] = wn * wn * (k * u - x1) - 2.0 * zeta * wn * x2;
    }

    public void step(double dt) {
        double[] dx = new double[2];
        derivatives(dx);
        x1 += dt * dx[0];
        x2 += dt * dx[1];
    }

    public double getOutput() {
        return x1;
    }

    public static void main(String[] args) {
        double wn = 5.0;
        double zeta = 0.4;
        double alpha = 50.0;
        double k = 1.0 / alpha;

        DominantSecondOrder sys = new DominantSecondOrder(wn, zeta, k);

        double dt = 0.001;
        double tEnd = 4.0;
        int steps = (int)(tEnd / dt);

        for (int i = 0; i <= steps; ++i) {
            double t = i * dt;
            sys.step(dt);
            if (i % (steps / 10) == 0) {
                System.out.println("t = " + t + "  y = " + sys.getOutput());
            }
        }
    }
}
