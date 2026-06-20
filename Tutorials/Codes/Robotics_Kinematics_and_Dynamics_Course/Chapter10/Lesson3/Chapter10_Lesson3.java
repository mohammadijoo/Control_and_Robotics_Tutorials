public class Pendulum {

    public static class Params {
        public double m;
        public double l;
        public double g;

        public Params(double m, double l, double g) {
            this.m = m;
            this.l = l;
            this.g = g;
        }
    }

    // Example torque profile (PD control)
    public static double torque(double t, double q, double qd) {
        double Kp = 5.0;
        double Kd = 1.0;
        return -Kp * q - Kd * qd;
    }

    // x = [q, qd], returns [qd, qdd]
    public static double[] rhs(double t, double[] x, Params p) {
        double q  = x[0];
        double qd = x[1];

        double tau = torque(t, q, qd);

        double denom = p.m * p.l * p.l;
        double qdd   = (tau - p.m * p.g * p.l * Math.sin(q)) / denom;

        return new double[]{qd, qdd};
    }
}
      
