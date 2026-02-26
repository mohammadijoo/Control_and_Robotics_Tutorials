public class FlexJointSimulation {

    static class Params {
        double J_l = 0.5, J_m = 0.05;
        double b_l = 0.02, b_m = 0.01;
        double k   = 150.0, d   = 0.5;
        double m   = 2.0,  ell = 0.4, g = 9.81;
        double tau0 = 1.0;
    }

    static void rhs(Params p, double t, double[] x, double[] xdot) {
        double q     = x[0];
        double qdot  = x[1];
        double th    = x[2];
        double thdot = x[3];

        double spring = p.k * (th - q);
        double damper = p.d * (thdot - qdot);
        double tau_m  = p.tau0;

        double qddot  = (spring + damper - p.m * p.g * p.ell * Math.sin(q) - p.b_l * qdot) / p.J_l;
        double thddot = (tau_m - spring - damper - p.b_m * thdot) / p.J_m;

        xdot[0] = qdot;
        xdot[1] = qddot;
        xdot[2] = thdot;
        xdot[3] = thddot;
    }

    public static void main(String[] args) {
        Params p = new Params();
        double[] x = {0.0, 0.0, 0.0, 0.0};
        double[] xdot = new double[4];

        double t  = 0.0;
        double dt = 0.0005;
        double T  = 2.0;

        while (t <= T) {
            rhs(p, t, x, xdot);
            for (int i = 0; i < 4; ++i) {
                x[i] += dt * xdot[i];
            }
            t += dt;
            System.out.println(t + " " + x[0] + " " + x[2]);
        }
    }
}
      
