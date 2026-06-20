import org.ejml.simple.SimpleMatrix;

public class TwoRDynamics {

    public static class Params {
        public double g, l1, l2, lc1, lc2, m1, m2, I1, I2;
    }

    public static SimpleMatrix M_sym(SimpleMatrix q, Params p) {
        double q2 = q.get(1);
        double c2 = Math.cos(q2);

        double M11 = p.I1 + p.I2
                + p.m1 * p.lc1 * p.lc1
                + p.m2 * (p.l1 * p.l1 + p.lc2 * p.lc2
                + 2.0 * p.l1 * p.lc2 * c2);

        double M12 = p.I2
                + p.m2 * (p.lc2 * p.lc2 + p.l1 * p.lc2 * c2);

        double M22 = p.I2 + p.m2 * p.lc2 * p.lc2;

        SimpleMatrix M = new SimpleMatrix(2, 2);
        M.set(0, 0, M11);
        M.set(0, 1, M12);
        M.set(1, 0, M12);
        M.set(1, 1, M22);
        return M;
    }

    public static SimpleMatrix g_sym(SimpleMatrix q, Params p) {
        double q1 = q.get(0);
        double q2 = q.get(1);
        double c1 = Math.cos(q1);
        double c12 = Math.cos(q1 + q2);

        double g1 = (p.m1 * p.lc1 + p.m2 * p.l1) * p.g * c1
                + p.m2 * p.lc2 * p.g * c12;
        double g2 = p.m2 * p.lc2 * p.g * c12;

        SimpleMatrix gVec = new SimpleMatrix(2, 1);
        gVec.set(0, 0, g1);
        gVec.set(1, 0, g2);
        return gVec;
    }

    public static void main(String[] args) {
        Params p = new Params();
        p.g = 9.81;
        p.l1 = 1.0; p.l2 = 1.0;
        p.lc1 = 0.5; p.lc2 = 0.5;
        p.m1 = 2.0; p.m2 = 1.0;
        p.I1 = 0.1; p.I2 = 0.1;

        SimpleMatrix q = new SimpleMatrix(2, 1, true, new double[]{0.3, -0.7});
        SimpleMatrix M = M_sym(q, p);
        SimpleMatrix gVec = g_sym(q, p);

        System.out.println("M(q) = ");
        M.print();
        System.out.println("g(q) = ");
        gVec.print();
    }
}
      
