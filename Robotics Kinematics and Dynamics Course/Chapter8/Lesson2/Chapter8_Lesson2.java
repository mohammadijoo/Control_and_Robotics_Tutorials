import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

public class ManipulabilityDemo {

    public static double yoshikawaManipulability(SimpleMatrix J, double tol) {
        SimpleSVD<SimpleMatrix> svd = J.svd();
        double[] s = svd.getSingularValues();

        double w = 1.0;
        int rank = 0;
        for (int i = 0; i < s.length; ++i) {
            if (s[i] > tol) {
                w *= s[i];
                ++rank;
            }
        }
        if (rank == 0) {
            return 0.0;
        }
        return w;
    }

    public static SimpleMatrix jacobian2R(double theta1, double theta2, double l1, double l2) {
        double s1 = Math.sin(theta1);
        double c1 = Math.cos(theta1);
        double s12 = Math.sin(theta1 + theta2);
        double c12 = Math.cos(theta1 + theta2);

        // Row-major initialization
        return new SimpleMatrix(2, 2, true,
                -l1 * s1 - l2 * s12, -l2 * s12,
                 l1 * c1 + l2 * c12,  l2 * c12);
    }

    public static void main(String[] args) {
        double l1 = 1.0;
        double l2 = 0.7;
        double theta1 = 0.5;
        double theta2 = 1.0;

        SimpleMatrix J = jacobian2R(theta1, theta2, l1, l2);
        double w = yoshikawaManipulability(J, 1e-6);

        System.out.println("J =");
        J.print();
        System.out.println("Yoshikawa manipulability w = " + w);
    }
}
      
