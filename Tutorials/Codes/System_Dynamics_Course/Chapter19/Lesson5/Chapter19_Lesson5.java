// Chapter19_Lesson5.java
// Discretized 1D heat equation + discrete-time Riccati iteration (DARE) using EJML
//
// Maven dependency (example):
//   org.ejml:ejml-simple:0.43

import org.ejml.simple.SimpleMatrix;

public class Chapter19_Lesson5 {

    static SimpleMatrix laplacianDirichlet(int N, double L) {
        double dx = L / (N + 1.0);
        SimpleMatrix D2 = new SimpleMatrix(N, N);
        for (int i = 0; i < N; i++) {
            D2.set(i, i, -2.0);
            if (i - 1 >= 0) D2.set(i, i - 1, 1.0);
            if (i + 1 < N)  D2.set(i, i + 1, 1.0);
        }
        return D2.divide(dx * dx);
    }

    static SimpleMatrix gridInterior(int N, double L) {
        double dx = L / (N + 1.0);
        SimpleMatrix x = new SimpleMatrix(N, 1);
        for (int i = 0; i < N; i++) x.set(i, 0, dx * (i + 1.0));
        return x;
    }

    static SimpleMatrix gaussianShape(SimpleMatrix x, double x0, double sigma) {
        int N = x.numRows();
        SimpleMatrix b = new SimpleMatrix(N, 1);
        for (int i = 0; i < N; i++) {
            double z = (x.get(i, 0) - x0) / sigma;
            b.set(i, 0, Math.exp(-0.5 * z * z));
        }
        return b;
    }

    static SimpleMatrix dareIterate(SimpleMatrix A, SimpleMatrix B,
                                   SimpleMatrix Q, SimpleMatrix R,
                                   int iters, double tol) {
        SimpleMatrix P = Q.copy();
        for (int k = 0; k < iters; k++) {
            SimpleMatrix BtPB = B.transpose().mult(P).mult(B);
            SimpleMatrix S = R.plus(BtPB);
            SimpleMatrix K = S.invert().mult(B.transpose().mult(P).mult(A)); // (m x n)
            SimpleMatrix Pnext = Q.plus(A.transpose().mult(P).mult(A))
                                  .minus(A.transpose().mult(P).mult(B).mult(K));
            double err = Pnext.minus(P).normF() / (P.normF() + 1e-12);
            P = Pnext;
            if (err < tol) break;
        }
        return P;
    }

    public static void main(String[] args) {
        final double alpha = 0.12;
        final double L = 1.0;
        final int N = 60;

        SimpleMatrix D2 = laplacianDirichlet(N, L);
        SimpleMatrix A = D2.scale(alpha);

        SimpleMatrix xgrid = gridInterior(N, L);
        SimpleMatrix b = gaussianShape(xgrid, 0.25, 0.07);
        b = b.divide(b.normF());
        SimpleMatrix B = b; // (N x 1)

        final double dt = 2e-3;
        SimpleMatrix Ad = SimpleMatrix.identity(N).plus(A.scale(dt));
        SimpleMatrix Bd = B.scale(dt);

        SimpleMatrix Q = SimpleMatrix.identity(N);
        SimpleMatrix R = new SimpleMatrix(1, 1);
        R.set(0, 0, 2e-3);

        SimpleMatrix P = dareIterate(Ad, Bd, Q, R, 20000, 1e-12);
        SimpleMatrix S = R.plus(Bd.transpose().mult(P).mult(Bd));
        SimpleMatrix K = S.invert().mult(Bd.transpose().mult(P).mult(Ad)); // (1 x N)

        // Initial condition
        SimpleMatrix x = new SimpleMatrix(N, 1);
        for (int i = 0; i < N; i++) {
            double z = xgrid.get(i, 0) - 0.75;
            x.set(i, 0, Math.exp(-80.0 * z * z));
        }

        int steps = (int)(4.0 / dt);
        double E0 = 0.5 * x.dot(x);
        for (int k = 0; k < steps; k++) {
            double u = -K.mult(x).get(0, 0);
            x = Ad.mult(x).plus(Bd.scale(u));
        }
        double E1 = 0.5 * x.dot(x);

        System.out.println("N=" + N + " dt=" + dt);
        System.out.println("Energy E(0)=" + E0 + "  E(end)=" + E1 + "  decay=" + (E1/(E0+1e-18)));
    }
}
