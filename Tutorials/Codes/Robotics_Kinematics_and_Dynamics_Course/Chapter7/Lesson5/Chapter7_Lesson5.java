import org.ejml.simple.SimpleMatrix;

public interface PoseFunction {
    double[] apply(double[] q);
}

public class NumericJacobian {

    public static SimpleMatrix numericJacobianVec(PoseFunction fk,
                                                  double[] q,
                                                  double h) {
        int n = q.length;
        double[] x0 = fk.apply(q);
        int m = x0.length;

        SimpleMatrix J = new SimpleMatrix(m, n);

        for (int i = 0; i < n; ++i) {
            double[] qp = q.clone();
            double[] qm = q.clone();
            qp[i] += h;
            qm[i] -= h;

            double[] xp = fk.apply(qp);
            double[] xm = fk.apply(qm);

            for (int r = 0; r < m; ++r) {
                double val = (xp[r] - xm[r]) / (2.0 * h);
                J.set(r, i, val);
            }
        }
        return J;
    }

    public static double checkJacobian(PoseFunction fk,
                                       java.util.function.Function<double[], SimpleMatrix> jacAna,
                                       double[] q,
                                       double h,
                                       double atol) {
        SimpleMatrix J_ana = jacAna.apply(q);
        SimpleMatrix J_num = numericJacobianVec(fk, q, h);
        SimpleMatrix diff  = J_num.minus(J_ana);

        double err = diff.normF();
        double ref = J_ana.normF() + atol;
        return err / ref;
    }
}
      
