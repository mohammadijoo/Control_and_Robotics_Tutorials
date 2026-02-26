import org.ejml.simple.SimpleMatrix;

public class Planar2R {

    public static SimpleMatrix jacobian2R(double q1, double q2,
                                          double l1, double l2) {
        double s1  = Math.sin(q1);
        double c1  = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[][] data = {
            { -l1 * s1 - l2 * s12, -l2 * s12 },
            {  l1 * c1 + l2 * c12,  l2 * c12 }
        };
        return new SimpleMatrix(data);
    }

    public static SimpleMatrix jdot2R(double q1, double q2,
                                      double dq1, double dq2,
                                      double l1, double l2) {
        double c1  = Math.cos(q1);
        double s1  = Math.sin(q1);
        double c12 = Math.cos(q1 + q2);
        double s12 = Math.sin(q1 + q2);

        double[][] data = {
            { -dq1 * l1 * c1 - (dq1 + dq2) * l2 * c12,
              -(dq1 + dq2) * l2 * c12 },
            { -dq1 * l1 * s1 - (dq1 + dq2) * l2 * s12,
              -(dq1 + dq2) * l2 * s12 }
        };
        return new SimpleMatrix(data);
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.7;
        double q1 = Math.toRadians(30.0);
        double q2 = Math.toRadians(20.0);
        double dq1 = 0.5, dq2 = -0.3;
        double ddq1 = 0.2, ddq2 = 0.1;

        SimpleMatrix J = jacobian2R(q1, q2, l1, l2);
        SimpleMatrix Jdot = jdot2R(q1, q2, dq1, dq2, l1, l2);

        SimpleMatrix dq = new SimpleMatrix(2, 1, true, new double[]{dq1, dq2});
        SimpleMatrix ddq = new SimpleMatrix(2, 1, true, new double[]{ddq1, ddq2});

        SimpleMatrix xdot = J.mult(dq);
        SimpleMatrix xddot = J.mult(ddq).plus(Jdot.mult(dq));

        System.out.println("xdot = " + xdot.transpose());
        System.out.println("xddot = " + xddot.transpose());
    }
}
      
