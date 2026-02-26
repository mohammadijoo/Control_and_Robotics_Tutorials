// Chapter2_Lesson4.java
// Autonomous Mobile Robots — Chapter 2 Lesson 4
// Omnidirectional Bases (mecanum, Swedish wheels)
//
// Dependencies (recommended):
//   - EJML (Efficient Java Matrix Library) for SVD/pseudoinverse
//
// If EJML is not available, you can implement a small 3x4 pseudoinverse via
// SVD or use a closed-form mapping for the standard mecanum geometry.

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class Chapter2_Lesson4 {

    // Build the 4x3 mecanum Jacobian J such that w = J * v, v=[vx vy omega]^T
    static DMatrixRMaj mecanumJacobian(double r, double lx, double ly, double alphaRad) {
        double a = lx + ly;
        double kappa = r * Math.cos(alphaRad);
        // Order: FL, FR, RL, RR
        double[] data = new double[] {
            1.0/kappa, -1.0/kappa, -a/kappa,
            1.0/kappa,  1.0/kappa,  a/kappa,
            1.0/kappa,  1.0/kappa, -a/kappa,
            1.0/kappa, -1.0/kappa,  a/kappa
        };
        return new DMatrixRMaj(4, 3, true, data);
    }

    static DMatrixRMaj inverseKinematics(DMatrixRMaj J, double vx, double vy, double omega) {
        DMatrixRMaj v = new DMatrixRMaj(3, 1, true, new double[]{vx, vy, omega});
        DMatrixRMaj w = new DMatrixRMaj(4, 1);
        CommonOps_DDRM.mult(J, v, w);
        return w;
    }

    static DMatrixRMaj forwardKinematicsLS(DMatrixRMaj J, DMatrixRMaj w) {
        // v = pinv(J) * w (least squares)
        DMatrixRMaj Jpinv = new DMatrixRMaj(3, 4);
        CommonOps_DDRM.pinv(J, Jpinv);
        DMatrixRMaj v = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.mult(Jpinv, w, v);
        return v;
    }

    public static void main(String[] args) {
        double r = 0.05, lx = 0.20, ly = 0.15, alpha = Math.PI / 4.0;

        DMatrixRMaj J = mecanumJacobian(r, lx, ly, alpha);

        double vx = 0.40, vy = -0.10, omega = 0.60;
        DMatrixRMaj w = inverseKinematics(J, vx, vy, omega);
        DMatrixRMaj vHat = forwardKinematicsLS(J, w);

        System.out.println("Wheel speeds w (rad/s):");
        w.print();

        System.out.println("Reconstructed twist vHat:");
        vHat.print();
    }
}
