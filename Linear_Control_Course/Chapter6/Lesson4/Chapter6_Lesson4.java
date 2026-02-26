public class ExtraPoleSimulation {

    public static void main(String[] args) {
        double zeta = 0.4;
        double wn   = 5.0;
        double a    = 20.0;

        double alpha2 = 2.0*zeta*wn + a;
        double alpha1 = wn*wn + 2.0*zeta*wn*a;
        double alpha0 = wn*wn*a;

        double kp = a;
        double k  = kp * wn*wn;

        double dt    = 0.0005;
        double tEnd  = 4.0;
        int    steps = (int)(tEnd / dt);

        // State x = [x1; x2; x3]
        double x1 = 0.0, x2 = 0.0, x3 = 0.0;
        double u  = 1.0;

        for (int i = 0; i <= steps; i++) {
            double t = i * dt;
            double y = k * x3;  // y = C x, with C = [0 0 k]

            System.out.println(t + " " + y);

            // xdot = A x + B u, with companion A and B = [0; 0; 1]
            double dx1 = x2;
            double dx2 = x3;
            double dx3 = -alpha0*x1 - alpha1*x2 - alpha2*x3 + u;

            x1 += dt * dx1;
            x2 += dt * dx2;
            x3 += dt * dx3;
        }
    }
}
