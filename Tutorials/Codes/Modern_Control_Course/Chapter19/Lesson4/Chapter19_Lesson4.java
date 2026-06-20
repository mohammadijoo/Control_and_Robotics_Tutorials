// Chapter19_Lesson4.java
// Identification of a minimal realization via sequential Kalman decomposition.
// Requires Apache Commons Math 3.6.1 on the classpath.
// Compile: javac -cp commons-math3-3.6.1.jar Chapter19_Lesson4.java
// Run:     java  -cp .:commons-math3-3.6.1.jar Chapter19_Lesson4

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

public class Chapter19_Lesson4 {
    static RealMatrix controllabilityMatrix(RealMatrix A, RealMatrix B) {
        int n = A.getRowDimension();
        int m = B.getColumnDimension();
        RealMatrix W = new Array2DRowRealMatrix(n, n * m);
        RealMatrix Ak = MatrixUtils.createRealIdentityMatrix(n);
        for (int k = 0; k < n; k++) {
            W.setSubMatrix(Ak.multiply(B).getData(), 0, k * m);
            Ak = Ak.multiply(A);
        }
        return W;
    }

    static RealMatrix observabilityMatrix(RealMatrix A, RealMatrix C) {
        int n = A.getRowDimension();
        int p = C.getRowDimension();
        RealMatrix W = new Array2DRowRealMatrix(n * p, n);
        RealMatrix Ak = MatrixUtils.createRealIdentityMatrix(n);
        for (int k = 0; k < n; k++) {
            W.setSubMatrix(C.multiply(Ak).getData(), k * p, 0);
            Ak = Ak.multiply(A);
        }
        return W;
    }

    static RealMatrix columns(RealMatrix M, int firstInclusive, int lastExclusive) {
        int rows = M.getRowDimension();
        int cols = lastExclusive - firstInclusive;
        RealMatrix out = new Array2DRowRealMatrix(rows, cols);
        for (int j = 0; j < cols; j++) {
            out.setColumnVector(j, M.getColumnVector(firstInclusive + j));
        }
        return out;
    }

    static RealMatrix hstack(RealMatrix A, RealMatrix B) {
        RealMatrix out = new Array2DRowRealMatrix(A.getRowDimension(), A.getColumnDimension() + B.getColumnDimension());
        out.setSubMatrix(A.getData(), 0, 0);
        out.setSubMatrix(B.getData(), 0, A.getColumnDimension());
        return out;
    }

    static RealMatrix sub(RealMatrix M, int r0, int r1, int c0, int c1) {
        return M.getSubMatrix(r0, r1 - 1, c0, c1 - 1);
    }

    static class MinimalResult {
        RealMatrix A, B, C, D;
        int reachableRank;
        int observableRankAfterReachable;
    }

    static MinimalResult kalmanMinimalRealization(RealMatrix A, RealMatrix B, RealMatrix C, RealMatrix D) {
        RealMatrix Wc = controllabilityMatrix(A, B);
        SingularValueDecomposition svdc = new SingularValueDecomposition(Wc);
        int rc = svdc.getRank();
        RealMatrix Tc = svdc.getU();

        RealMatrix Ac = Tc.transpose().multiply(A).multiply(Tc);
        RealMatrix Bc = Tc.transpose().multiply(B);
        RealMatrix Cc = C.multiply(Tc);

        RealMatrix Ar = sub(Ac, 0, rc, 0, rc);
        RealMatrix Br = sub(Bc, 0, rc, 0, B.getColumnDimension());
        RealMatrix Cr = sub(Cc, 0, C.getRowDimension(), 0, rc);

        RealMatrix Wo = observabilityMatrix(Ar, Cr);
        SingularValueDecomposition svdo = new SingularValueDecomposition(Wo);
        int ro = svdo.getRank();
        RealMatrix V = svdo.getV();

        RealMatrix To;
        if (ro < rc) {
            RealMatrix unobservable = columns(V, ro, rc);
            RealMatrix observableComplement = columns(V, 0, ro);
            To = hstack(unobservable, observableComplement);
        } else {
            To = columns(V, 0, ro);
        }

        RealMatrix Ao = To.transpose().multiply(Ar).multiply(To);
        RealMatrix Bo = To.transpose().multiply(Br);
        RealMatrix Co = Cr.multiply(To);

        int nUnobs = rc - ro;
        MinimalResult result = new MinimalResult();
        result.A = sub(Ao, nUnobs, nUnobs + ro, nUnobs, nUnobs + ro);
        result.B = sub(Bo, nUnobs, nUnobs + ro, 0, B.getColumnDimension());
        result.C = sub(Co, 0, C.getRowDimension(), nUnobs, nUnobs + ro);
        result.D = D.copy();
        result.reachableRank = rc;
        result.observableRankAfterReachable = ro;
        return result;
    }

    static double transferAtRealS(RealMatrix A, RealMatrix B, RealMatrix C, RealMatrix D, double s) {
        int n = A.getRowDimension();
        RealMatrix sIminusA = MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(s).subtract(A);
        RealMatrix X = new LUDecomposition(sIminusA).getSolver().solve(B);
        return C.multiply(X).add(D).getEntry(0, 0);
    }

    static void printMatrix(String name, RealMatrix M) {
        System.out.println(name + ":");
        for (int i = 0; i < M.getRowDimension(); i++) {
            for (int j = 0; j < M.getColumnDimension(); j++) {
                System.out.printf("% .6f ", M.getEntry(i, j));
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        RealMatrix A = MatrixUtils.createRealMatrix(new double[][]{
            {0.0, 1.0, 0.0, 0.0},
            {-2.0, -3.0, 0.0, 0.0},
            {0.0, 0.0, -4.0, 0.0},
            {0.0, 0.0, 0.0, -5.0}
        });
        RealMatrix B = MatrixUtils.createColumnRealMatrix(new double[]{0.0, 1.0, 1.0, 0.0});
        RealMatrix C = MatrixUtils.createRowRealMatrix(new double[]{1.0, 0.0, 0.0, 2.0});
        RealMatrix D = MatrixUtils.createRealMatrix(new double[][]{{0.0}});

        MinimalResult minsys = kalmanMinimalRealization(A, B, C, D);
        System.out.println("rank controllability = " + minsys.reachableRank);
        System.out.println("rank observability after reachable reduction = " + minsys.observableRankAfterReachable);
        printMatrix("Amin", minsys.A);
        printMatrix("Bmin", minsys.B);
        printMatrix("Cmin", minsys.C);

        double[] sValues = {0.0, 1.0, 2.0};
        for (double s : sValues) {
            double gf = transferAtRealS(A, B, C, D, s);
            double gm = transferAtRealS(minsys.A, minsys.B, minsys.C, minsys.D, s);
            System.out.printf("s = %.1f, Gfull = %.8f, Gmin = %.8f, error = %.2e%n", s, gf, gm, Math.abs(gf - gm));
        }
    }
}
