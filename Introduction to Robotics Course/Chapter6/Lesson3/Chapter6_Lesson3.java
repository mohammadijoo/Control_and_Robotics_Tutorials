public class HydraulicCylinderSim {

    static final double A = 3.0e-3;
    static final double beta_e = 1.2e9;
    static final double Vt = 6.0e-5;
    static final double m = 8.0;
    static final double b = 120.0;
    static final double Kq = 2.5e-5;
    static final double Kp = 1.0e-11;
    static final double Ct = 5.0e-12;

    static double uFunc(double t) {
        return (t >= 0.1) ? 2.0 : 0.0;
    }

    public static void main(String[] args) {
        double x = 0.0, xd = 0.0, p = 0.0;
        double dt = 1e-4;
        double T = 1.0;

        for (double t = 0.0; t <= T; t += dt) {
            double u = uFunc(t);
            double FL = 0.0;

            double q = Kq * u - Kp * p;
            double pd = (beta_e / Vt) * (q - A * xd - Ct * p);
            double xdd = (A * p - b * xd - FL) / m;

            x  += xd * dt;
            xd += xdd * dt;
            p  += pd * dt;

            if (((int)(t/dt)) % 1000 == 0) {
                System.out.println(t + " " + x + " " + p);
            }
        }
    }
}
