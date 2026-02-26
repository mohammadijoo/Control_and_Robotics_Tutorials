public class SaturationSimulation {
    static double sat(double u, double uMax) {
        if (u > uMax) return uMax;
        if (u < -uMax) return -uMax;
        return u;
    }

    public static void main(String[] args) {
        double omegaN = 1.0;
        double zeta = 0.2;
        double K = 8.0;
        double uMax = 2.0;

        double tFinal = 20.0;
        double dt = 1e-3;
        int nSteps = (int) (tFinal / dt);

        double x1 = 0.0;
        double x2 = 0.0;
        double t = 0.0;

        System.out.println("t,y,u");

        for (int k = 0; k <= nSteps; ++k) {
            double y = x1;
            double r = 1.0;
            double e = r - y;
            double v = K * e;
            double u = sat(v, uMax);

            System.out.println(t + "," + y + "," + u);

            double dx1 = x2;
            double dx2 = -2.0 * zeta * omegaN * x2
                        - omegaN * omegaN * x1
                        + omegaN * omegaN * u;

            x1 += dt * dx1;
            x2 += dt * dx2;
            t  += dt;
        }
    }
}
