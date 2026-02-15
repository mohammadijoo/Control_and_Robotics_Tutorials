public class Avoidance2R {

    public static double[][] jacobian2R(double[] q,
                                        double l1, double l2) {
        double q1 = q[0], q2 = q[1];
        double s1  = Math.sin(q1);
        double c1  = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[][] J = new double[2][2];
        J[0][0] = -l1 * s1 - l2 * s12;
        J[0][1] = -l2 * s12;
        J[1][0] =  l1 * c1 + l2 * c12;
        J[1][1] =  l2 * c12;
        return J;
    }

    public static double manipulability(double[][] J) {
        // JJ^T for 2x2 case
        double a = J[0][0] * J[0][0] + J[0][1] * J[0][1];
        double b = J[0][0] * J[1][0] + J[0][1] * J[1][1];
        double d = J[1][0] * J[1][0] + J[1][1] * J[1][1];

        double det = a * d - b * b;
        return Math.sqrt(det);
    }

    public static double jointLimitPenalty(double[] q,
                                           double[] qmin,
                                           double[] qmax) {
        double phi = 0.0;
        for (int i = 0; i < q.length; ++i) {
            double qi = q[i];
            double ql = qmin[i];
            double qu = qmax[i];
            phi += 1.0 / Math.pow(qi - ql, 2.0)
                 + 1.0 / Math.pow(qu - qi, 2.0);
        }
        return phi;
    }

    public static double workspacePenalty(double[] q,
                                          double l1, double l2,
                                          double epsWs) {
        double q1 = q[0], q2 = q[1];
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        double r = Math.hypot(x, y);

        double rmin = Math.abs(l1 - l2);
        double rmax = l1 + l2;
        double dWs = Math.min(r - rmin, rmax - r);
        return 1.0 / (dWs * dWs + epsWs);
    }

    public static double avoidanceCost(double[] q,
                                       double l1, double l2,
                                       double eps) {
        double[][] J = jacobian2R(q, l1, l2);
        double w = manipulability(J);
        double phiSing = 1.0 / (w * w + eps);

        double[] qmin = new double[]{-Math.PI, -Math.PI};
        double[] qmax = new double[]{ Math.PI,  Math.PI};
        double phiJoint = jointLimitPenalty(q, qmin, qmax);
        double phiWs = workspacePenalty(q, l1, l2, 1e-3);

        return phiSing + phiJoint + phiWs;
    }

    public static void main(String[] args) {
        double[] q = new double[]{0.0, 0.5};
        for (int k = 0; k < 10; ++k) {
            double c = avoidanceCost(q, 1.0, 1.0, 1e-3);
            System.out.println("Iter " + k + ", cost = " + c);
            // Offline gradient-based update of q can be added as needed
        }
    }
}
      
