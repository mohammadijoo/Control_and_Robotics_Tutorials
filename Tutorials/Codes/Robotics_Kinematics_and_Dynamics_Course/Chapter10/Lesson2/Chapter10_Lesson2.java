import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class Planar2RGeneralizedForces {

    public static DMatrixRMaj jacobian2R(double q1, double q2,
                                         double l1, double l2) {
        double s1  = Math.sin(q1);
        double c1  = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        DMatrixRMaj J = new DMatrixRMaj(2, 2);
        J.set(0, 0, -l1 * s1 - l2 * s12);
        J.set(0, 1, -l2 * s12);
        J.set(1, 0,  l1 * c1 + l2 * c12);
        J.set(1, 1,  l2 * c12);
        return J;
    }

    public static DMatrixRMaj generalizedTorquesFromForce(
            double q1, double q2,
            double l1, double l2,
            DMatrixRMaj f) {

        DMatrixRMaj J = jacobian2R(q1, q2, l1, l2);
        DMatrixRMaj Jt = new DMatrixRMaj(2, 2);
        CommonOps_DDRM.transpose(J, Jt);

        DMatrixRMaj tau = new DMatrixRMaj(2, 1);
        CommonOps_DDRM.mult(Jt, f, tau);
        return tau;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.8;
        double q1 = 0.5, q2 = -0.3;

        DMatrixRMaj f = new DMatrixRMaj(2, 1);
        f.set(0, 0, 10.0);
        f.set(1, 0, 5.0);

        DMatrixRMaj tau = generalizedTorquesFromForce(q1, q2, l1, l2, f);
        System.out.println("Joint torques:");
        System.out.println(tau.get(0, 0) + ", " + tau.get(1, 0));
    }
}
      
