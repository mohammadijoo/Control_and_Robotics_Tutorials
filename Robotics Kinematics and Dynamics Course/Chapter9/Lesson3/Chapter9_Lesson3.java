import org.ejml.simple.SimpleMatrix;

public class JacobianTranspose2R {

    public static SimpleMatrix jacobian2R(double q1, double q2,
                                          double l1, double l2) {
        double s1  = Math.sin(q1);
        double c1  = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[][] data = new double[][] {
            { -l1 * s1 - l2 * s12, -l2 * s12 },
            {  l1 * c1 + l2 * c12,  l2 * c12 }
        };
        return new SimpleMatrix(data);
    }

    public static SimpleMatrix torqueFromForce(
            double q1, double q2,
            double Fx, double Fy,
            double l1, double l2) {

        SimpleMatrix J = jacobian2R(q1, q2, l1, l2);
        SimpleMatrix f = new SimpleMatrix(2, 1, true, new double[] {Fx, Fy});
        return J.transpose().mult(f);
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 1.0;
        double q1 = 0.0, q2 = 0.0;
        double Fx = 0.0, Fy = -10.0;

        SimpleMatrix tau = torqueFromForce(q1, q2, Fx, Fy, l1, l2);
        System.out.println("tau = ");
        tau.print();
    }
}
      
