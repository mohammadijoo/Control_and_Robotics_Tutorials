public class TwoLinkEnergy {

    public static class Params {
        public double m1, m2;
        public double l1, l2;
        public double c1, c2;
        public double I1, I2;
        public double g = 9.81;
    }

    public static double[][] inertiaMatrix(double[] q, Params p) {
        double q1 = q[0];
        double q2 = q[1];

        double cos2 = Math.cos(q2);

        double M11 = p.I1 + p.I2
                   + p.m1 * p.c1 * p.c1
                   + p.m2 * (p.l1 * p.l1 + p.c2 * p.c2
                             + 2.0 * p.l1 * p.c2 * cos2);
        double M12 = p.I2 + p.m2 * (p.c2 * p.c2 + p.l1 * p.c2 * cos2);
        double M22 = p.I2 + p.m2 * p.c2 * p.c2;

        double[][] M = new double[2][2];
        M[0][0] = M11; M[0][1] = M12;
        M[1][0] = M12; M[1][1] = M22;
        return M;
    }

    public static double potentialEnergy(double[] q, Params p) {
        double q1 = q[0];
        double q2 = q[1];
        return p.m1 * p.g * p.c1 * Math.sin(q1)
             + p.m2 * p.g * (p.l1 * Math.sin(q1)
                             + p.c2 * Math.sin(q1 + q2));
    }

    public static double kineticEnergy(double[] q, double[] qdot, Params p) {
        double[][] M = inertiaMatrix(q, p);
        // qdot^T M qdot for 2x2
        double v0 = qdot[0];
        double v1 = qdot[1];
        double T = 0.5 * (v0 * (M[0][0] * v0 + M[0][1] * v1)
                        + v1 * (M[1][0] * v0 + M[1][1] * v1));
        return T;
    }

    public static void main(String[] args) {
        Params p = new Params();
        p.m1 = 2.0; p.m2 = 1.5;
        p.l1 = 0.4; p.l2 = 0.3;
        p.c1 = 0.2; p.c2 = 0.15;
        p.I1 = 0.02; p.I2 = 0.01;

        double[] q = {0.5, -0.3};
        double[] qdot = {0.4, 0.2};

        double T = kineticEnergy(q, qdot, p);
        double V = potentialEnergy(q, p);
        double E = T + V;

        System.out.println("T = " + T + ", V = " + V + ", E = " + E);
    }
}
      
