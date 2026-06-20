public class PneumaticAxisSim {
    public static void main(String[] args) {
        double M = 2.0, A = 3.0e-4, V0 = 2.0e-5, k = 1.2;
        double R = 287.0, T = 293.0, b = 15.0;
        double pAtm = 101325.0, Kq = 1.3e-4, Kp = 2.0e-9;

        double x = 0.0, v = 0.0, p = pAtm;

        double dt = 1e-4, tf = 0.4;
        for (double t = 0.0; t < tf; t += dt) {
            double V = V0 + A*x;
            double u = (t >= 0.05) ? 1.0 : 0.0;
            double mdot = Kq*u - Kp*(p - pAtm);

            double pdot = (k*R*T/V)*mdot - (k*p/V)*(A*v);
            double F = A*p - b*v;
            double vdot = F/M;

            x += v*dt;
            v += vdot*dt;
            p += pdot*dt;
        }

        System.out.println("Final x (m): " + x);
        System.out.println("Final p (kPa): " + p/1000.0);
    }
}
