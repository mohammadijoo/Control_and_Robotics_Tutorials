import org.ejml.simple.SimpleMatrix;

public class Planar2RCond {

    static SimpleMatrix jacobian2R(double q1, double q2,
                                   double l1, double l2) {
        double s1 = Math.sin(q1);
        double c1 = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[][] data = new double[][]{
            { -l1 * s1 - l2 * s12, -l2 * s12 },
            {  l1 * c1 + l2 * c12,  l2 * c12 }
        };
        return new SimpleMatrix(data);
    }

    static double cond2(SimpleMatrix J) {
        var svd = J.svd();
        double[] s = svd.getW().extractDiag().getDDRM().getData();
        double sigmaMax = Math.max(s[0], s[1]);
        double sigmaMin = Math.min(s[0], s[1]);
        return sigmaMax / sigmaMin;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 1.0;
        double q1 = 0.0, q2 = 0.9;

        SimpleMatrix J = jacobian2R(q1, q2, l1, l2);
        double kappa = cond2(J);

        System.out.println("J =");
        J.print();
        System.out.println("kappa_2(J) = " + kappa);
    }
}
      
