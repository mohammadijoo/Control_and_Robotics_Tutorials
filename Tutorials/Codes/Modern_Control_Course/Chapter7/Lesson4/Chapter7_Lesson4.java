// Chapter7_Lesson4.java
/*
Modern Control — Chapter 7, Lesson 4
Output Response Using x(t) and y(t) = Cx(t) + Du(t)

This Java example uses EJML (Efficient Java Matrix Library) to compute
an approximation of exp(A t) via eigen-decomposition (diagonalization):
    A = V Λ V^{-1}  (when A is diagonalizable)
    exp(A t) = V exp(Λ t) V^{-1}

Then, for constant input u(t)=u0 and invertible A:
    x(t) = exp(A t) x0 + A^{-1}(exp(A t) - I) B u0
    y(t) = C x(t) + D u0

If eigen-decomposition fails (e.g., defective A), the code falls back to
a truncated series exp(A t) ≈ Σ_{k=0}^K (A t)^k / k! (educational).

Dependency (Gradle/Maven):
  org.ejml:ejml-all:0.43+

Run:
  javac -cp ejml-all-0.43.jar Chapter7_Lesson4.java
  java  -cp .:ejml-all-0.43.jar Chapter7_Lesson4
*/

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SpecializedOps_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;

public class Chapter7_Lesson4 {

    static DMatrixRMaj expmSeries(DMatrixRMaj A, double t, int K) {
        int n = A.numRows;
        DMatrixRMaj At = A.copy();
        CommonOps_DDRM.scale(t, At);

        DMatrixRMaj X = CommonOps_DDRM.identity(n); // term 0
        DMatrixRMaj term = CommonOps_DDRM.identity(n);

        for (int k = 1; k <= K; k++) {
            DMatrixRMaj tmp = new DMatrixRMaj(n, n);
            CommonOps_DDRM.mult(term, At, tmp);
            term = tmp;
            CommonOps_DDRM.scale(1.0 / k, term);     // divide by k iteratively => /k!
            CommonOps_DDRM.addEquals(X, term);
        }
        return X;
    }

    static DMatrixRMaj expmEigen(DMatrixRMaj A, double t) {
        int n = A.numRows;
        EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(n, true);
        if (!eig.decompose(A)) {
            return null;
        }

        DMatrixRMaj V = new DMatrixRMaj(n, n);
        DMatrixRMaj Vinv = new DMatrixRMaj(n, n);
        DMatrixRMaj LambdaExp = new DMatrixRMaj(n, n);

        // Build V and exp(Lambda t). This handles only real eigenpairs cleanly.
        for (int i = 0; i < n; i++) {
            if (eig.getEigenvalue(i).isComplex()) {
                return null;
            }
            double lam = eig.getEigenvalue(i).getReal();
            DMatrixRMaj vi = eig.getEigenVector(i);
            if (vi == null) return null;
            for (int r = 0; r < n; r++) {
                V.set(r, i, vi.get(r, 0));
            }
            LambdaExp.set(i, i, Math.exp(lam * t));
        }

        // Vinv = inv(V)
        if (!CommonOps_DDRM.invert(V, Vinv)) {
            return null;
        }

        DMatrixRMaj tmp = new DMatrixRMaj(n, n);
        DMatrixRMaj Phi = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(V, LambdaExp, tmp);
        CommonOps_DDRM.mult(tmp, Vinv, Phi);
        return Phi;
    }

    public static void main(String[] args) {
        // Example: 2-state SISO system
        DMatrixRMaj A = new DMatrixRMaj(new double[][]{
                {0.0, 1.0},
                {-2.0, -3.0}
        });
        DMatrixRMaj B = new DMatrixRMaj(new double[][]{
                {0.0},
                {1.0}
        });
        DMatrixRMaj C = new DMatrixRMaj(new double[][]{
                {1.0, 0.0}
        });
        DMatrixRMaj D = new DMatrixRMaj(new double[][]{
                {0.25}
        });

        DMatrixRMaj x0 = new DMatrixRMaj(new double[][]{
                {1.0},
                {0.0}
        });
        DMatrixRMaj u0 = new DMatrixRMaj(new double[][]{
                {1.0}
        });

        // Invert A (for constant-input formula)
        DMatrixRMaj Ainv = A.copy();
        if (!CommonOps_DDRM.invert(Ainv)) {
            System.out.println("A is not invertible; cannot use constant-input closed form.");
            return;
        }

        double[] times = new double[]{0.0, 0.5, 1.0, 2.0, 6.0};

        System.out.println("y(t) for constant u(t)=1:");
        for (double t : times) {
            DMatrixRMaj Phi = expmEigen(A, t);
            if (Phi == null) {
                Phi = expmSeries(A, t, 30);
            }

            // x(t) = Phi x0 + A^{-1}(Phi - I) B u0
            DMatrixRMaj I = CommonOps_DDRM.identity(2);
            DMatrixRMaj PhiMinusI = new DMatrixRMaj(2, 2);
            CommonOps_DDRM.subtract(Phi, I, PhiMinusI);

            DMatrixRMaj tmp1 = new DMatrixRMaj(2, 1);
            CommonOps_DDRM.mult(Phi, x0, tmp1);

            DMatrixRMaj tmp2 = new DMatrixRMaj(2, 1);
            DMatrixRMaj tmp3 = new DMatrixRMaj(2, 1);
            CommonOps_DDRM.mult(PhiMinusI, B, tmp2);
            CommonOps_DDRM.mult(Ainv, tmp2, tmp3);
            CommonOps_DDRM.multAdd(tmp3, u0, tmp1); // tmp1 += tmp3*u0

            DMatrixRMaj y = new DMatrixRMaj(1, 1);
            CommonOps_DDRM.mult(C, tmp1, y);
            CommonOps_DDRM.multAdd(D, u0, y);

            System.out.printf("t=%.2f, y=%.8f%n", t, y.get(0,0));
        }
    }
}
