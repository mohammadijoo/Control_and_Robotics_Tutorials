public class LoopShapingServo {
    public static void main(String[] args) {
        double J  = 0.01;
        double b  = 0.1;
        double K  = 1.0;

        double Kp = 20.0;
        double Ki = 40.0;

        double dt    = 0.0005;
        double tEnd  = 1.0;
        int steps    = (int)(tEnd / dt);

        double x1 = 0.0;   // position
        double x2 = 0.0;   // velocity
        double integralError = 0.0;
        double ref = 1.0;

        double[] y = new double[steps];

        for (int k = 0; k < steps; ++k) {
            double e = ref - x1;
            integralError += e * dt;
            double u = Kp * e + Ki * integralError;

            double x2dot = (K * u - b * x2) / J;
            double x1dot = x2;

            x1 += dt * x1dot;
            x2 += dt * x2dot;

            y[k] = x1;
        }

        System.out.println("Final position: " + y[steps - 1]);

        // Java-based robotics frameworks (e.g. certain mobile-robot and competition libraries)
        // use similar update loops for servo control, often with abstractions for sensors,
        // actuators, and timing.
    }
}
