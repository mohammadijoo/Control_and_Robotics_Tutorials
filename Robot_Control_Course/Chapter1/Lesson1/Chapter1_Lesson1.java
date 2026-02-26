
public class JointControlSim {
    public static void main(String[] args) {
        double Kp = 5.0;
        double tau = 0.2;
        double Ts = 0.001;
        double T_end = 1.0;
        int nSteps = (int)(T_end / Ts);

        double[] t = new double[nSteps + 1];
        double[] r = new double[nSteps + 1];
        double[] y = new double[nSteps + 1];
        double[] u = new double[nSteps + 1];

        for (int k = 0; k <= nSteps; ++k) {
            t[k] = k * Ts;
            r[k] = 1.0; // step
            y[k] = 0.0;
            u[k] = 0.0;
        }

        for (int k = 0; k < nSteps; ++k) {
            double e = r[k] - y[k];
            u[k] = Kp * e;
            double dy = (-y[k] + u[k]) / tau;
            y[k + 1] = y[k] + Ts * dy;
        }

        for (int k = 0; k < nSteps; k += nSteps / 10) {
            System.out.println("t=" + t[k] + "  y=" + y[k]);
        }
    }
}
