import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class DHKinematics {

    public static DMatrixRMaj dhTransform(double theta, double d,
                                          double a, double alpha) {
        double ct = Math.cos(theta);
        double st = Math.sin(theta);
        double ca = Math.cos(alpha);
        double sa = Math.sin(alpha);

        DMatrixRMaj T = new DMatrixRMaj(4,4);
        T.set(0,0, ct);      T.set(0,1, -st * ca);  T.set(0,2,  st * sa);  T.set(0,3, a * ct);
        T.set(1,0, st);      T.set(1,1,  ct * ca);  T.set(1,2, -ct * sa);  T.set(1,3, a * st);
        T.set(2,0, 0.0);     T.set(2,1,        sa); T.set(2,2,       ca);  T.set(2,3, d);
        T.set(3,0, 0.0);     T.set(3,1,       0.0); T.set(3,2,      0.0);  T.set(3,3, 1.0);
        return T;
    }

    public static DMatrixRMaj forwardKinematicsDH(double[][] dhTable) {
        DMatrixRMaj T = CommonOps_DDRM.identity(4);
        for (double[] row : dhTable) {
            double theta = row[0];
            double d     = row[1];
            double a     = row[2];
            double alpha = row[3];
            DMatrixRMaj Ti = dhTransform(theta, d, a, alpha);
            DMatrixRMaj temp = new DMatrixRMaj(4,4);
            CommonOps_DDRM.mult(T, Ti, temp);
            T = temp;
        }
        return T;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.7;
        double q1 = 0.5, q2 = -0.3;
        double[][] dh = {
            { q1, 0.0, l1, 0.0 },
            { q2, 0.0, l2, 0.0 }
        };
        DMatrixRMaj T = forwardKinematicsDH(dh);
        System.out.println("T_0_2:");
        T.print();
    }
}
      
