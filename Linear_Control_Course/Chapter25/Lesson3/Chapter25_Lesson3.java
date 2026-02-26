public class CascadeOuterLoopExample {
    public static void main(String[] args) {
        double k_i = 9.0;
        double k_p = 0.5;
        double k_I = 1.0;

        double h = 1e-3;
        double tEnd = 5.0;
        int nSteps = (int) (tEnd / h);

        double x = 0.0;   // plant state
        double z = 0.0;   // PI integrator
        double y = 0.0;   // output
        double r = 1.0;   // step reference

        double[] tHist = new double[nSteps];
        double[] yHist = new double[nSteps];

        for (int k = 0; k < nSteps; ++k) {
            double t = k * h;

            double eOuter = r - y;
            z += h * (k_I * eOuter);
            double w = k_p * eOuter + z;

            double u = k_i * (w - y);  // inner P loop

            double xDot = -x + u;
            x += h * xDot;
            y = x;

            tHist[k] = t;
            yHist[k] = y;
        }

        for (int i = 0; i < 10; ++i) {
            int idx = i * (nSteps / 10);
            System.out.println("t=" + tHist[idx] + "  y=" + yHist[idx]);
        }
    }
}
