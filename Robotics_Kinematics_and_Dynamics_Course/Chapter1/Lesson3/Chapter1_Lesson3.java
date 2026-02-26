import org.ejml.simple.SimpleMatrix;

public class TwoRGrad {

    public static SimpleMatrix fk2R(double q1, double q2,
                                    double l1, double l2) {
        double c1 = Math.cos(q1);
        double s1 = Math.sin(q1);
        double c12 = Math.cos(q1 + q2);
        double s12 = Math.sin(q1 + q2);

        double x = l1 * c1 + l2 * c12;
        double y = l1 * s1 + l2 * s12;

        return new SimpleMatrix(2, 1, true, new double[]{x, y});
    }

    public static SimpleMatrix jac2R(double q1, double q2,
                                     double l1, double l2) {
        double c1 = Math.cos(q1);
        double s1 = Math.sin(q1);
        double c12 = Math.cos(q1 + q2);
        double s12 = Math.sin(q1 + q2);

        double[] data = new double[]{
            -l1 * s1 - l2 * s12,  -l2 * s12,
             l1 * c1 + l2 * c12,   l2 * c12
        };
        return new SimpleMatrix(2, 2, true, data);
    }

    // grad_phi = J(q)^T * (f(q) - x_d)
    public static SimpleMatrix gradPhi2R(SimpleMatrix q,
                                         SimpleMatrix x_d,
                                         double l1, double l2) {
        double q1 = q.get(0);
        double q2 = q.get(1);

        SimpleMatrix x = fk2R(q1, q2, l1, l2);
        SimpleMatrix e = x.minus(x_d);
        SimpleMatrix J = jac2R(q1, q2, l1, l2);
        return J.transpose().mult(e);
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.8;
        SimpleMatrix q = new SimpleMatrix(2, 1, true, new double[]{0.5, -0.3});
        SimpleMatrix x_d = new SimpleMatrix(2, 1, true, new double[]{1.2, 0.3});

        SimpleMatrix g = gradPhi2R(q, x_d, l1, l2);
        System.out.println("grad_phi(q) = ");
        g.print();
    }
}
      
