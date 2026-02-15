public class IK2R {

    static double[] fk2R(double[] q, double l1, double l2) {
        double q1 = q[0];
        double q2 = q[1];
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        return new double[] { x, y };
    }

    static double[][] jacobian2R(double[] q, double l1, double l2) {
        double q1 = q[0];
        double q2 = q[1];
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

    // Solve 2x2 system A x = b
    static double[] solve2x2(double[][] A, double[] b) {
        double a11 = A[0][0], a12 = A[0][1];
        double a21 = A[1][0], a22 = A[1][1];
        double det = a11 * a22 - a12 * a21;
        if (Math.abs(det) < 1e-12) {
            throw new RuntimeException("Singular matrix");
        }
        double inv11 =  a22 / det;
        double inv12 = -a12 / det;
        double inv21 = -a21 / det;
        double inv22 =  a11 / det;
        double x0 = inv11 * b[0] + inv12 * b[1];
        double x1 = inv21 * b[0] + inv22 * b[1];
        return new double[] { x0, x1 };
    }

    static boolean ikNewton2R(double[] xd, double l1, double l2,
                              double[] q, double tol, int maxIter, double alpha) {
        for (int k = 0; k < maxIter; ++k) {
            double[] x = fk2R(q, l1, l2);
            double rx = x[0] - xd[0];
            double ry = x[1] - xd[1];
            double err = Math.sqrt(rx * rx + ry * ry);
            if (err < tol) {
                return true;
            }
            double[][] J = jacobian2R(q, l1, l2);
            double[] minusR = new double[] { -rx, -ry };
            double[] dq = solve2x2(J, minusR);
            q[0] += alpha * dq[0];
            q[1] += alpha * dq[1];
        }
        return false;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.8;
        double[] xd = { 1.2, 0.5 };
        double[] q0 = { 0.0, 0.0 };
        double[] q = q0.clone();
        boolean ok = ikNewton2R(xd, l1, l2, q, 1e-6, 50, 1.0);
        System.out.println("IK success: " + ok +
                           " q = [" + q[0] + ", " + q[1] + "]");
    }
}
      
