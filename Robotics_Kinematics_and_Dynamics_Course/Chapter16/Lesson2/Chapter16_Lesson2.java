import org.ejml.simple.SimpleMatrix;

public class Planar3RPR {

    public static class Geometry {
        public double[][] B = new double[3][2];
        public double[][] P = new double[3][2];
    }

    private static SimpleMatrix rot2(double phi) {
        double c = Math.cos(phi);
        double s = Math.sin(phi);
        double[][] data = { {c, -s},
                           {s,  c} };
        return new SimpleMatrix(data);
    }

    private static SimpleMatrix S() {
        double[][] data = { {0.0, -1.0},
                           {1.0,  0.0} };
        return new SimpleMatrix(data);
    }

    public static void jacobians3RPR(
            Geometry geom,
            double[] L,
            double[] pose,
            SimpleMatrix Jq,
            SimpleMatrix Jx,
            SimpleMatrix Jf)
    {
        double x = pose[0];
        double y = pose[1];
        double phi = pose[2];

        SimpleMatrix p = new SimpleMatrix(2, 1, true, new double[]{x, y});
        SimpleMatrix R = rot2(phi);
        SimpleMatrix S = S();

        // J_q: diagonal
        Jq.zero();
        for (int i = 0; i < 3; ++i) {
            Jq.set(i, i, -2.0 * L[i]);
        }

        // J_x rows
        for (int i = 0; i < 3; ++i) {
            SimpleMatrix Bi = new SimpleMatrix(2, 1, true,
                    new double[]{geom.B[i][0], geom.B[i][1]});
            SimpleMatrix Pi = new SimpleMatrix(2, 1, true,
                    new double[]{geom.P[i][0], geom.P[i][1]});

            SimpleMatrix d = p.plus(R.mult(Pi)).minus(Bi);
            double dix = d.get(0, 0);
            double diy = d.get(1, 0);

            SimpleMatrix dphi = S.mult(R.mult(Pi));

            double dPhi_dx = 2.0 * dix;
            double dPhi_dy = 2.0 * diy;
            double dPhi_dphi = 2.0 * d.dot(dphi);

            Jx.set(i, 0, dPhi_dx);
            Jx.set(i, 1, dPhi_dy);
            Jx.set(i, 2, dPhi_dphi);
        }

        // J_f = - J_x^{-1} J_q
        SimpleMatrix JxInv = Jx.invert();
        SimpleMatrix JfLocal = JxInv.mult(Jq).scale(-1.0);
        Jf.set(JfLocal);
    }
}
      
