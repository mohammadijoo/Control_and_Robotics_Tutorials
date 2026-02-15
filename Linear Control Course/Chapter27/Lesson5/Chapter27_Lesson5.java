public class PiTrackingSimulation {

    public static void main(String[] args) {
        double Kp = 3.2;
        double Ki = 9.0;

        double dt = 0.001;
        double tFinal = 8.0;
        int nSteps = (int) (tFinal / dt) + 1;

        double[] t = new double[nSteps];
        double[] y = new double[nSteps];
        double[] z = new double[nSteps];
        double[] u = new double[nSteps];
        double[] d = new double[nSteps];

        for (int k = 0; k < nSteps; ++k) {
            t[k] = k * dt;
        }

        for (int k = 0; k < nSteps - 1; ++k) {
            double r = 1.0; // unit step
            double dist = (t[k] >= 3.0) ? 0.2 : 0.0;
            double e = r - y[k];

            // PI controller
            u[k] = Kp * e + Ki * z[k];

            double dz = e;
            double dy = -y[k] + u[k] + dist;

            z[k + 1] = z[k] + dt * dz;
            y[k + 1] = y[k] + dt * dy;
            d[k] = dist;
        }
        // final sample
        double rEnd = 1.0;
        double distEnd = (t[nSteps - 1] >= 3.0) ? 0.2 : 0.0;
        double eEnd = rEnd - y[nSteps - 1];
        u[nSteps - 1] = Kp * eEnd + Ki * z[nSteps - 1];
        d[nSteps - 1] = distEnd;

        System.out.println("Final output y(T) = " + y[nSteps - 1]);

        // In a robotics Java stack, the same PI computation would run
        // in a periodic control task with joint states as feedback.
    }
}
