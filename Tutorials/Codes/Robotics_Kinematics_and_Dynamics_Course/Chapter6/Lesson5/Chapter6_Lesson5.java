import org.ejml.simple.SimpleMatrix;

public class RedundantIK {

    public static SimpleMatrix dampedPseudoinverse(SimpleMatrix J, double lambda) {
        int m = J.numRows();
        SimpleMatrix JJt = J.mult(J.transpose());
        SimpleMatrix I = SimpleMatrix.identity(m);
        SimpleMatrix inv = JJt.plus(I.scale(lambda * lambda)).invert();
        return J.transpose().mult(inv);
    }

    public static SimpleMatrix planar3Jacobian(double[] q, double[] L) {
        double q1 = q[0], q2 = q[1], q3 = q[2];
        double l1 = L[0], l2 = L[1], l3 = L[2];

        double s1 = Math.sin(q1), c1 = Math.cos(q1);
        double s12 = Math.sin(q1 + q2), c12 = Math.cos(q1 + q2);
        double s123 = Math.sin(q1 + q2 + q3), c123 = Math.cos(q1 + q2 + q3);

        SimpleMatrix J = new SimpleMatrix(2, 3);
        J.set(0, 0, -l1 * s1 - l2 * s12 - l3 * s123);
        J.set(1, 0,  l1 * c1 + l2 * c12 + l3 * c123);
        J.set(0, 1, -l2 * s12 - l3 * s123);
        J.set(1, 1,  l2 * c12 + l3 * c123);
        J.set(0, 2, -l3 * s123);
        J.set(1, 2,  l3 * c123);
        return J;
    }

    public static SimpleMatrix jointLimitGradient(double[] q,
                                                  double[] qMin,
                                                  double[] qMax) {
        int n = q.length;
        SimpleMatrix grad = new SimpleMatrix(n, 1);
        for (int i = 0; i < n; ++i) {
            double qMid = 0.5 * (qMin[i] + qMax[i]);
            double span = qMax[i] - qMin[i];
            double normed = (q[i] - qMid) / span;
            grad.set(i, 0, normed / span);
        }
        return grad;
    }

    public static SimpleMatrix redundancyResolutionStep(double[] q,
                                                        double[] xdotDes,
                                                        double[] L,
                                                        double[] qMin,
                                                        double[] qMax,
                                                        double alpha,
                                                        double lambda) {
        SimpleMatrix J = planar3Jacobian(q, L);
        SimpleMatrix Jpinv = dampedPseudoinverse(J, lambda);

        SimpleMatrix xdot = new SimpleMatrix(2, 1, true, xdotDes);
        SimpleMatrix qdot0 = Jpinv.mult(xdot);

        SimpleMatrix I = SimpleMatrix.identity(J.numCols());
        SimpleMatrix N = I.minus(Jpinv.mult(J));

        SimpleMatrix gradH = jointLimitGradient(q, qMin, qMax);
        SimpleMatrix qdotH = N.mult(gradH).scale(-alpha);

        return qdot0.plus(qdotH);
    }

    public static void main(String[] args) {
        double[] L = {0.4, 0.3, 0.2};
        double[] q = {0.0, 0.3, -0.2};
        double[] qMin = {-Math.PI, -Math.PI, -Math.PI};
        double[] qMax = { Math.PI,  Math.PI,  Math.PI};
        double[] xdotDes = {0.05, 0.0};

        SimpleMatrix qdot = redundancyResolutionStep(q, xdotDes,
                                                     L, qMin, qMax,
                                                     0.2, 1e-3);

        System.out.println("qdot = ");
        qdot.print();
    }
}
      
