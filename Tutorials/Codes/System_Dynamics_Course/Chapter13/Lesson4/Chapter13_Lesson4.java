/*
Chapter13_Lesson4.java
Damping in MDOF Systems and Mode Shapes with Damping

Demonstrates:
1) Undamped generalized eigenproblem (symmetric) via Cholesky transform:
      K phi = (w^2) M phi
   with M = L L^T, transform to A = L^{-1} K L^{-T}, solve A y = (w^2) y, then phi = L^{-T} y.
2) Rayleigh damping coefficients from two target modal damping ratios.
3) Non-proportional damping: state-space eigenanalysis to obtain complex poles.

Library:
  EJML (Efficient Java Matrix Library)
  Maven coordinates (example):
    org.ejml:ejml-all:0.43

This is a single-file demo for teaching; in production, organize into packages.
*/

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.Complex_F64;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

import java.util.ArrayList;
import java.util.Comparator;

public class Chapter13_Lesson4 {

    static class PoleInfo {
        double sigma, wd, wn, zeta;
        PoleInfo(double sigma, double wd, double wn, double zeta) {
            this.sigma = sigma; this.wd = wd; this.wn = wn; this.zeta = zeta;
        }
    }

    static void chainMatrices3DOF(DMatrixRMaj M, DMatrixRMaj K) {
        double m1 = 1.2, m2 = 1.0, m3 = 0.8;
        double k1 = 2500.0, k2 = 1800.0, k3 = 1200.0;

        M.reshape(3,3);
        K.reshape(3,3);
        CommonOps_DDRM.fill(M, 0.0);
        CommonOps_DDRM.fill(K, 0.0);

        M.set(0,0, m1); M.set(1,1, m2); M.set(2,2, m3);

        K.set(0,0, k1+k2); K.set(0,1, -k2);
        K.set(1,0, -k2);   K.set(1,1, k2+k3); K.set(1,2, -k3);
        K.set(2,1, -k3);   K.set(2,2, k3);
    }

    static DMatrixRMaj massNormalizeModes(DMatrixRMaj M, DMatrixRMaj Phi) {
        int n = Phi.getNumRows();
        int r = Phi.getNumCols();
        DMatrixRMaj PhiN = Phi.copy();
        DMatrixRMaj tmp = new DMatrixRMaj(n,1);
        for (int i = 0; i < r; i++) {
            // mi = phi_i^T M phi_i
            CommonOps_DDRM.extract(PhiN, 0, n, i, i+1, tmp, 0, 0);
            DMatrixRMaj Mphi = new DMatrixRMaj(n,1);
            CommonOps_DDRM.mult(M, tmp, Mphi);
            double mi = CommonOps_DDRM.dot(tmp, Mphi);
            double scale = 1.0/Math.sqrt(mi);
            for (int k=0; k<n; k++) {
                PhiN.set(k, i, PhiN.get(k, i)*scale);
            }
        }
        return PhiN;
    }

    static double[] rayleighFromTwoTargets(double omegaI, double zetaI, double omegaJ, double zetaJ) {
        // alpha + beta*omega^2 = 2*zeta*omega
        double A11 = 1.0, A12 = omegaI*omegaI;
        double A21 = 1.0, A22 = omegaJ*omegaJ;
        double b1 = 2.0*zetaI*omegaI;
        double b2 = 2.0*zetaJ*omegaJ;

        double det = A11*A22 - A12*A21;
        double alpha = ( b1*A22 - A12*b2 )/det;
        double beta  = ( A11*b2 - b1*A21 )/det;
        return new double[]{alpha, beta};
    }

    static DMatrixRMaj buildStateMatrix(DMatrixRMaj M, DMatrixRMaj C, DMatrixRMaj K) {
        int n = M.getNumRows();
        DMatrixRMaj Minv = new DMatrixRMaj(n,n);
        CommonOps_DDRM.invert(M, Minv);

        DMatrixRMaj A = new DMatrixRMaj(2*n, 2*n);
        CommonOps_DDRM.fill(A, 0.0);

        // top-right I
        for (int i=0; i<n; i++) A.set(i, n+i, 1.0);

        // bottom-left -Minv*K
        DMatrixRMaj MinvK = new DMatrixRMaj(n,n);
        CommonOps_DDRM.mult(Minv, K, MinvK);
        CommonOps_DDRM.scale(-1.0, MinvK);
        CommonOps_DDRM.insert(MinvK, A, n, 0);

        // bottom-right -Minv*C
        DMatrixRMaj MinvC = new DMatrixRMaj(n,n);
        CommonOps_DDRM.mult(Minv, C, MinvC);
        CommonOps_DDRM.scale(-1.0, MinvC);
        CommonOps_DDRM.insert(MinvC, A, n, n);

        return A;
    }

    public static void main(String[] args) {
        DMatrixRMaj M = new DMatrixRMaj(3,3);
        DMatrixRMaj K = new DMatrixRMaj(3,3);
        chainMatrices3DOF(M, K);

        // Cholesky of M: M = L L^T
        CholeskyDecomposition_F64<DMatrixRMaj> chol = DecompositionFactory_DDRM.chol(M.numRows, true);
        if (!chol.decompose(M.copy())) {
            throw new RuntimeException("Cholesky failed; M must be SPD.");
        }
        DMatrixRMaj L = chol.getT(null); // lower triangular

        // Compute Linv
        DMatrixRMaj Linv = new DMatrixRMaj(3,3);
        CommonOps_DDRM.invert(L, Linv);

        // Transform A = Linv * K * Linv^T (symmetric)
        DMatrixRMaj tmp = new DMatrixRMaj(3,3);
        DMatrixRMaj A = new DMatrixRMaj(3,3);
        CommonOps_DDRM.mult(Linv, K, tmp);
        DMatrixRMaj LinvT = new DMatrixRMaj(3,3);
        CommonOps_DDRM.transpose(Linv, LinvT);
        CommonOps_DDRM.mult(tmp, LinvT, A);

        // Eigen decomposition of symmetric A -> omega^2
        EigenDecomposition_F64<DMatrixRMaj> eigSym = DecompositionFactory_DDRM.eig(3, true);
        if (!eigSym.decompose(A)) {
            throw new RuntimeException("Eigen decomposition failed.");
        }

        double[] omega = new double[3];
        DMatrixRMaj Y = new DMatrixRMaj(3,3); // eigenvectors
        for (int i=0; i<3; i++) {
            Complex_F64 ev = eigSym.getEigenvalue(i);
            omega[i] = Math.sqrt(ev.getReal());
            DMatrixRMaj vi = eigSym.getEigenVector(i);
            for (int r=0; r<3; r++) Y.set(r, i, vi.get(r,0));
        }

        // Back-transform modes: Phi = L^{-T} Y
        DMatrixRMaj LT = new DMatrixRMaj(3,3);
        CommonOps_DDRM.transpose(L, LT);
        DMatrixRMaj LTinv = new DMatrixRMaj(3,3);
        CommonOps_DDRM.invert(LT, LTinv);
        DMatrixRMaj Phi = new DMatrixRMaj(3,3);
        CommonOps_DDRM.mult(LTinv, Y, Phi);

        Phi = massNormalizeModes(M, Phi);

        System.out.println("Undamped omegas (rad/s):");
        for (double w : omega) System.out.printf("%.6f  ", w);
        System.out.println("\n");

        // Rayleigh damping from mode 1 and mode 3 targets
        double[] ab = rayleighFromTwoTargets(omega[0], 0.02, omega[2], 0.05);
        double alpha = ab[0], beta = ab[1];

        DMatrixRMaj C_ray = new DMatrixRMaj(3,3);
        DMatrixRMaj alphaM = M.copy();
        DMatrixRMaj betaK  = K.copy();
        CommonOps_DDRM.scale(alpha, alphaM);
        CommonOps_DDRM.scale(beta, betaK);
        CommonOps_DDRM.add(alphaM, betaK, C_ray);

        // Cm = Phi^T C Phi
        DMatrixRMaj PhiT = new DMatrixRMaj(3,3);
        CommonOps_DDRM.transpose(Phi, PhiT);
        DMatrixRMaj Cm = new DMatrixRMaj(3,3);
        DMatrixRMaj tmp2 = new DMatrixRMaj(3,3);
        CommonOps_DDRM.mult(PhiT, C_ray, tmp2);
        CommonOps_DDRM.mult(tmp2, Phi, Cm);

        System.out.printf("Rayleigh alpha=%.6f, beta=%.6f%n", alpha, beta);
        System.out.println("Modal damping matrix (Rayleigh), Phi^T C Phi:");
        Cm.print();

        // Non-proportional damping example
        DMatrixRMaj C_np = new DMatrixRMaj(3,3);
        CommonOps_DDRM.fill(C_np, 0.0);
        double c13 = 45.0, c2g = 35.0;
        C_np.set(0,0, C_np.get(0,0)+c13);
        C_np.set(2,2, C_np.get(2,2)+c13);
        C_np.set(0,2, C_np.get(0,2)-c13);
        C_np.set(2,0, C_np.get(2,0)-c13);
        C_np.set(1,1, C_np.get(1,1)+c2g);

        // Build state matrix and compute eigenvalues (complex poles)
        DMatrixRMaj Astate = buildStateMatrix(M, C_np, K);
        EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(Astate.getNumRows(), false);
        if (!eig.decompose(Astate)) throw new RuntimeException("State eigen decomposition failed.");

        ArrayList<Complex_F64> poles = new ArrayList<>();
        for (int i=0; i<eig.getNumberOfEigenvalues(); i++) {
            Complex_F64 ev = eig.getEigenvalue(i);
            if (ev.getImaginary() > 1e-8) poles.add(ev);
        }
        poles.sort(Comparator.comparingDouble(Complex_F64::getImaginary));

        System.out.println("Complex poles (sigma + j*wd), first three modes:");
        for (int i=0; i<Math.min(3, poles.size()); i++) {
            double sigma = poles.get(i).getReal();
            double wd    = poles.get(i).getImaginary();
            double wn    = Math.sqrt(sigma*sigma + wd*wd);
            double zeta  = (wn > 0) ? (-sigma/wn) : 0.0;
            System.out.printf("Mode %d: sigma=%+.6f, wd=%.6f, wn=%.6f, zeta=%.6f%n",
                    (i+1), sigma, wd, wn, zeta);
        }
    }
}
