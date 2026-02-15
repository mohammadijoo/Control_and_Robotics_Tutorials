
import org.ejml.simple.SimpleMatrix;

public class HierarchicalWBC {

    public static SimpleMatrix dampedPseudoInverse(SimpleMatrix J, double lambda) {
        int m = J.numRows();
        int n = J.numCols();
        double lam2 = lambda * lambda;

        if (m >= n) {
            SimpleMatrix A = J.transpose().mult(J)
                .plus(SimpleMatrix.identity(n).scale(lam2));
            return A.pseudoInverse().mult(J.transpose());
        } else {
            SimpleMatrix A = J.mult(J.transpose())
                .plus(SimpleMatrix.identity(m).scale(lam2));
            return J.transpose().mult(A.pseudoInverse());
        }
    }

    public static SimpleMatrix nullspaceProjector(SimpleMatrix J, double lambda) {
        int n = J.numCols();
        SimpleMatrix Jpinv = dampedPseudoInverse(J, lambda);
        SimpleMatrix I = SimpleMatrix.identity(n);
        return I.minus(Jpinv.mult(J));
    }

    public static SimpleMatrix twoTaskVelocityControl(SimpleMatrix J1,
                                                      SimpleMatrix xdot1_star,
                                                      SimpleMatrix J2,
                                                      SimpleMatrix xdot2_star,
                                                      double lambda) {
        // Task 1
        SimpleMatrix J1pinv = dampedPseudoInverse(J1, lambda);
        SimpleMatrix qdot1 = J1pinv.mult(xdot1_star);

        // Null space of Task 1
        SimpleMatrix N1 = nullspaceProjector(J1, lambda);

        // Effective Task 2 in null space
        SimpleMatrix J2_eff = J2.mult(N1);
        SimpleMatrix J2_eff_pinv = dampedPseudoInverse(J2_eff, lambda);

        SimpleMatrix residual2 = xdot2_star.minus(J2.mult(qdot1));
        SimpleMatrix qdot2_corr = N1.mult(J2_eff_pinv.mult(residual2));

        return qdot1.plus(qdot2_corr);
    }

    public static void main(String[] args) {
        int n_dof = 7;
        int m1 = 3;
        int m2 = 3;

        SimpleMatrix J1 = SimpleMatrix.random_DDRM(m1, n_dof, -1.0, 1.0, null);
        SimpleMatrix J2 = SimpleMatrix.random_DDRM(m2, n_dof, -1.0, 1.0, null);

        SimpleMatrix xdot1_star = new SimpleMatrix(m1, 1, true, new double[]{0.1, 0.0, -0.05});
        SimpleMatrix xdot2_star = new SimpleMatrix(m2, 1, true, new double[]{0.0, 0.05, 0.0});

        double lambda = 1e-3;
        SimpleMatrix qdot = twoTaskVelocityControl(J1, xdot1_star, J2, xdot2_star, lambda);
        qdot.print();
    }
}
