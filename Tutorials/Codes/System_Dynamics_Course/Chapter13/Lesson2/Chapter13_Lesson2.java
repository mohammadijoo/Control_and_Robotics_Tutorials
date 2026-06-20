/*
Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
Lesson 1: Natural Frequencies and Normal Modes (Eigenvalue Problems)

This example uses EJML (Efficient Java Matrix Library) to solve the generalized symmetric eigenproblem:
    K phi = (w^2) M phi
by converting it to a standard symmetric eigenproblem via Cholesky:
    M = L L^T
    A = L^{-1} K L^{-T}  (symmetric)
    A u = (w^2) u
    phi = L^{-T} u

Dependency (Gradle):
    implementation 'org.ejml:ejml-all:0.43'

Compile/run:
    javac -cp ejml-all-0.43.jar Chapter13_Lesson1.java
    java  -cp .;ejml-all-0.43.jar Chapter13_Lesson1
(Use ':' instead of ';' on Linux/macOS.)
*/

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;

import java.util.Arrays;

public class Chapter13_Lesson1 {

    static DMatrixRMaj diag(double[] d) {
        int n = d.length;
        DMatrixRMaj M = new DMatrixRMaj(n, n);
        for (int i = 0; i < n; i++) M.set(i, i, d[i]);
        return M;
    }

    // Build 3-DOF chain: wall-k1-m1-k2-m2-k3-m3-(free end)
    static DMatrixRMaj buildK(double[] springs) {
        int n = springs.length;
        DMatrixRMaj K = new DMatrixRMaj(n, n);

        for (int i = 0; i < n; i++) {
            K.add(i, i, springs[i]);
            if (i > 0) {
                K.add(i, i, springs[i-1]);
                K.add(i, i-1, -springs[i-1]);
                K.add(i-1, i, -springs[i-1]);
            }
        }
        return K;
    }

    static class Modes {
        double[] w;       // natural frequencies (rad/s)
        DMatrixRMaj Phi;  // columns are mass-normalized modes
        double[] w2;
    }

    static Modes generalizedModes(DMatrixRMaj M, DMatrixRMaj K) {
        int n = M.numRows;

        // Cholesky: M = L L^T (lower)
        CholeskyDecompositionInner_DDRM chol = new CholeskyDecompositionInner_DDRM(true);
        if (!chol.decompose(M.copy())) {
            throw new RuntimeException("Cholesky failed: M not SPD?");
        }
        DMatrixRMaj L = chol.getT(null); // lower-triangular

        // Compute Linv = inv(L)
        DMatrixRMaj Linv = new DMatrixRMaj(n, n);
        CommonOps_DDRM.invert(L, Linv);

        // A = Linv * K * Linv^T
        DMatrixRMaj temp = new DMatrixRMaj(n, n);
        DMatrixRMaj A = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(Linv, K, temp);
        CommonOps_DDRM.multTransB(temp, Linv, A); // temp * Linv^T

        // Symmetric eigen decomposition
        EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(n, true);
        if (!eig.decompose(A)) {
            throw new RuntimeException("Eigen decomposition failed.");
        }

        // Extract eigenpairs (not guaranteed sorted)
        double[] w2 = new double[n];
        DMatrixRMaj U = new DMatrixRMaj(n, n); // eigenvectors u
        for (int i = 0; i < n; i++) {
            w2[i] = eig.getEigenvalue(i).getReal();
            DMatrixRMaj ui = eig.getEigenVector(i);
            for (int r = 0; r < n; r++) U.set(r, i, ui.get(r, 0));
        }

        // Sort by w2 ascending
        Integer[] idx = new Integer[n];
        for (int i = 0; i < n; i++) idx[i] = i;
        Arrays.sort(idx, (a, b) -> Double.compare(w2[a], w2[b]));

        double[] w2s = new double[n];
        DMatrixRMaj Us = new DMatrixRMaj(n, n);
        for (int col = 0; col < n; col++) {
            int j = idx[col];
            w2s[col] = Math.max(0.0, w2[j]);
            for (int r = 0; r < n; r++) Us.set(r, col, U.get(r, j));
        }

        // phi = L^{-T} u => solve L^T phi = u
        DMatrixRMaj Phi = new DMatrixRMaj(n, n);
        DMatrixRMaj Lt = new DMatrixRMaj(n, n);
        CommonOps_DDRM.transpose(L, Lt);
        // Phi = inv(L^T) * U
        DMatrixRMaj Ltinv = new DMatrixRMaj(n, n);
        CommonOps_DDRM.invert(Lt, Ltinv);
        CommonOps_DDRM.mult(Ltinv, Us, Phi);

        // Mass-normalize columns: phi_i^T M phi_i = 1
        DMatrixRMaj tempv = new DMatrixRMaj(n, 1);
        for (int i = 0; i < n; i++) {
            // tempv = M * phi
            DMatrixRMaj phi = CommonOps_DDRM.extract(Phi, 0, n, i, i+1);
            CommonOps_DDRM.mult(M, phi, tempv);
            double mi = dot(phi, tempv);
            scaleColumn(Phi, i, 1.0 / Math.sqrt(mi));
        }

        double[] w = new double[n];
        for (int i = 0; i < n; i++) w[i] = Math.sqrt(w2s[i]);

        Modes out = new Modes();
        out.w = w;
        out.w2 = w2s;
        out.Phi = Phi;
        return out;
    }

    static double dot(DMatrixRMaj a, DMatrixRMaj b) {
        double s = 0.0;
        for (int i = 0; i < a.numRows; i++) s += a.get(i, 0) * b.get(i, 0);
        return s;
    }

    static void scaleColumn(DMatrixRMaj A, int col, double s) {
        for (int r = 0; r < A.numRows; r++) A.set(r, col, A.get(r, col) * s);
    }

    static void printMatrix(String name, DMatrixRMaj A) {
        System.out.println(name + ":");
        for (int i = 0; i < A.numRows; i++) {
            for (int j = 0; j < A.numCols; j++) {
                System.out.printf("%12.6f ", A.get(i, j));
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[] masses  = {2.0, 1.5, 1.0};
        double[] springs = {200.0, 300.0, 250.0};

        DMatrixRMaj M = diag(masses);
        DMatrixRMaj K = buildK(springs);

        Modes modes = generalizedModes(M, K);

        System.out.println("Natural frequencies (rad/s):");
        for (int i = 0; i < modes.w.length; i++) {
            System.out.printf("  w%d = %.6f%n", i+1, modes.w[i]);
        }

        // Verify orthogonality: Phi^T M Phi = I and Phi^T K Phi = diag(w^2)
        DMatrixRMaj Mt = new DMatrixRMaj(3, 3);
        DMatrixRMaj Kt = new DMatrixRMaj(3, 3);

        DMatrixRMaj temp = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.multTransA(modes.Phi, M, temp);     // Phi^T M
        CommonOps_DDRM.mult(temp, modes.Phi, Mt);          // (Phi^T M) Phi

        CommonOps_DDRM.multTransA(modes.Phi, K, temp);     // Phi^T K
        CommonOps_DDRM.mult(temp, modes.Phi, Kt);          // (Phi^T K) Phi

        printMatrix("\nPhi^T M Phi (should be I)", Mt);
        printMatrix("\nPhi^T K Phi (should be diag(w^2))", Kt);
    }
}
