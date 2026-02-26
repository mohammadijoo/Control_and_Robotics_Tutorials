public class SecondOrderSystem {

    private final double omegaN;
    private final double zeta;

    public SecondOrderSystem(double omegaN, double zeta) {
        this.omegaN = omegaN;
        this.zeta = zeta;
    }

    public static class Response {
        public final double[] t;
        public final double[] y;

        public Response(double[] t, double[] y) {
            this.t = t;
            this.y = y;
        }
    }

    // Simulate unit-step response y(t) for t in [0, tFinal] with step h
    public Response simulateStep(double tFinal, double h) {
        int nSteps = (int) Math.round(tFinal / h);
        double[] t = new double[nSteps + 1];
        double[] y = new double[nSteps + 1];

        double x1 = 0.0; // y
        double x2 = 0.0; // dy/dt

        for (int k = 0; k <= nSteps; ++k) {
            t[k] = k * h;
            y[k] = x1;

            double u = 1.0; // unit step
            double dx1 = x2;
            double dx2 = -2.0 * zeta * omegaN * x2
                         - omegaN * omegaN * x1
                         + omegaN * omegaN * u;

            x1 += h * dx1;
            x2 += h * dx2;
        }
        return new Response(t, y);
    }

    public static void main(String[] args) {
        SecondOrderSystem sys = new SecondOrderSystem(20.0, 0.4);
        Response resp = sys.simulateStep(4.0, 0.001);

        // Print first few samples (can be plotted in MATLAB, Python, etc.)
        for (int k = 0; k < resp.t.length; ++k) {
            System.out.println(resp.t[k] + " " + resp.y[k]);
        }
    }
}
