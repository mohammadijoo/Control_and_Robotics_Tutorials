import org.ejml.simple.SimpleMatrix;

public class Planar3RRedundant {

    public static SimpleMatrix jacobian(double[] q, double[] L) {
        double q1 = q[0], q2 = q[1], q3 = q[2];
        double l1 = L[0], l2 = L[1], l3 = L[2];

        double s1 = Math.sin(q1), c1 = Math.cos(q1);
        double s12 = Math.sin(q1 + q2), c12 = Math.cos(q1 + q2);
        double s123 = Math.sin(q1 + q2 + q3), c123 = Math.cos(q1 + q2 + q3);

        double dx_dq1 = -l1 * s1 - l2 * s12 - l3 * s123;
        double dx_dq2 = -l2 * s12 - l3 * s123;
        double dx_dq3 = -l3 * s123;

        double dy_dq1 =  l1 * c1 + l2 * c12 + l3 * c123;
        double dy_dq2 =  l2 * c12 + l3 * c123;
        double dy_dq3 =  l3 * c123;

        double[][] data = {
            {dx_dq1, dx_dq2, dx_dq3},
            {dy_dq1, dy_dq2, dy_dq3}
        };
        return new SimpleMatrix(data);  // 2x3
    }

    public static SimpleMatrix minimumNormQdot(double[] q, double[] L,
                                               double[] xdotDes) {
        SimpleMatrix J = jacobian(q, L);
        SimpleMatrix xdot = new SimpleMatrix(2, 1, true, xdotDes);
        // EJML provides a pseudoInverse() method
        SimpleMatrix J_pinv = J.pseudoInverse(); // 3x2
        return J_pinv.mult(xdot);                // 3x1
    }

    public static void main(String[] args) {
        double[] L = {0.5, 0.4, 0.3};
        double[] q = {0.2, 0.5, -0.4};
        double[] xdotDes = {0.05, -0.02};

        SimpleMatrix qdot = minimumNormQdot(q, L, xdotDes);
        qdot.print("qdot:");
    }
}
      
