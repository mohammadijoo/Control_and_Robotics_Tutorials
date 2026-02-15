public class PendulumDynamics {

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

    // qddot = (tau - m g l sin(q)) / (m l^2)
    public static double accel(double q, double qdot, double tau, Params p) {
        double M = p.m * p.l * p.l;
        double G = p.m * p.g * p.l * Math.sin(q);
        double Cqdot = 0.0;
        return (tau - Cqdot - G) / M;
    }

    public static void main(String[] args) {
        Params p = new Params(1.0, 1.0, 9.81);
        double q = 0.5;
        double qdot = 0.0;
        double tau = 0.0;
        double qddot = accel(q, qdot, tau, p);
        System.out.println("qddot = " + qddot);
    }
}
      
