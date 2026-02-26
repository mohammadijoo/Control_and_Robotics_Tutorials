public class ErrorSignalSimulation {
    public static void main(String[] args) {
        double K = 5.0;
        double dt = 0.001;
        double tEnd = 10.0;
        int N = (int) (tEnd / dt);

        double x1 = 0.0;
        double x2 = 0.0;

        double[] eStep = new double[N];
        double[] eRamp = new double[N];

        // STEP input r(t) = 1
        x1 = 0.0;
        x2 = 0.0;
        for (int k = 0; k < N; ++k) {
            double t = k * dt;
            double r = 1.0;
            double y = x1;
            double e = r - y;
            double u = e;

            double x1Dot = x2;
            double x2Dot = -x2 + K * u;

            x1 += dt * x1Dot;
            x2 += dt * x2Dot;

            eStep[k] = e;
        }

        // RAMP input r(t) = t
        x1 = 0.0;
        x2 = 0.0;
        for (int k = 0; k < N; ++k) {
            double t = k * dt;
            double r = t;
            double y = x1;
            double e = r - y;
            double u = e;

            double x1Dot = x2;
            double x2Dot = -x2 + K * u;

            x1 += dt * x1Dot;
            x2 += dt * x2Dot;

            eRamp[k] = e;
        }

        System.out.println("# k  eStep  eRamp");
        for (int k = 0; k < N; ++k) {
            System.out.println(k + " " + eStep[k] + " " + eRamp[k]);
        }

        // In Java-based robotics libraries such as WPILib, the error e is
        // computed similarly and passed to PID controllers for motor commands.
    }
}
