public class PDServoHF {
    private double d, k, Kp, Kd;
    private double omegaH; // high-frequency pole

    public PDServoHF(double d, double k, double Kp, double Kd, double omegaH) {
        this.d = d;
        this.k = k;
        this.Kp = Kp;
        this.Kd = Kd;
        this.omegaH = omegaH;
    }

    public double[] simulate(double m, double tEnd, double dt) {
        double x = 0.0;   // position
        double v = 0.0;   // velocity
        double z = 0.0;   // state of unmodeled pole (first-order filter)
        double r = 1.0;   // step reference

        int N = (int) Math.round(tEnd / dt) + 1;
        double[] y = new double[N];
        int idx = 0;

        for (double t = 0.0; t <= tEnd; t += dt) {
            double e = r - x;
            double edot = -v;
            double u = Kp * e + Kd * edot;

            // unmodeled high-frequency pole: z' = -omegaH * z + omegaH * u
            double zdot = -omegaH * z + omegaH * u;
            z += zdot * dt;

            // plant dynamics: m x'' + d x' + k x = z
            double a = (z - d * v - k * x) / m;
            v += a * dt;
            x += v * dt;

            y[idx++] = x;
        }
        return y;
    }
}
