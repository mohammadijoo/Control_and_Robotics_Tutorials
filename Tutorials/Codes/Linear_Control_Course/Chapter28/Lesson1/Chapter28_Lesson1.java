public class SecondOrderStepEuler {
    private final double zeta;
    private final double wn;

    public SecondOrderStepEuler(double zeta, double wn) {
        this.zeta = zeta;
        this.wn = wn;
    }

    public double[] simulateStep(double tFinal, double h) {
        int nSteps = (int) Math.round(tFinal / h);
        double[] y = new double[nSteps + 1];

        double x1 = 0.0; // position
        double x2 = 0.0; // velocity
        double u = 1.0;  // unit step

        for (int k = 0; k <= nSteps; ++k) {
            y[k] = x1;

            double dx1 = x2;
            double dx2 = -2.0 * zeta * wn * x2 - wn * wn * x1 + wn * wn * u;

            x1 += h * dx1;
            x2 += h * dx2;
        }
        return y;
    }

    public static void main(String[] args) {
        SecondOrderStepEuler sim = new SecondOrderStepEuler(0.5, 4.0);
        double[] y = sim.simulateStep(5.0, 0.001);
        System.out.println("Final value y(T) = " + y[y.length - 1]);
    }
}
