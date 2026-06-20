public class LagControlledProcess {

    // In robotics (e.g., FRC with WPILib), similar loops are executed
    // periodically in the main robot control thread.

    public static void main(String[] args) {
        double Kp = 2.0;
        double Tp = 5.0;

        double Kc = 24.5;
        double beta = 22.0;
        double Tl = 25.0;

        double h = 0.01;
        double tEnd = 50.0;
        double r = 1.0;

        double x = 0.0;  // process state
        double w = 0.0;  // lag filter state
        double y;
        double e;
        double u;

        int steps = (int) (tEnd / h);

        for (int k = 0; k < steps; ++k) {
            double t = k * h;

            y = x;
            e = r - y;

            // Lag filter ODE: beta * Tl * dw/dt + w = e
            double dw = (e - w) / (beta * Tl);
            w += h * dw;

            // Control law
            u = Kc * ((1.0 - 1.0 / beta) * w + (1.0 / beta) * e);

            // Process dynamics
            double dx = -(1.0 / Tp) * x + (Kp / Tp) * u;
            x += h * dx;

            if (k % 100 == 0) {
                System.out.println(t + " " + y);
            }
        }
    }
}
