/*
Chapter24_Lesson5.java
Modern Control - Chapter 24, Lesson 5
Design examples for multi-input pole placement using EJML.

Dependency:
    EJML Simple API
Maven dependency:
    org.ejml:ejml-simple:0.43 or newer
*/

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

import java.util.Random;

public class Chapter24_Lesson5 {
    static SimpleMatrix controllabilityMatrix(SimpleMatrix A, SimpleMatrix B) {
        int n = A.numRows();
        int m = B.numCols();
        SimpleMatrix C = new SimpleMatrix(n, n * m);
        SimpleMatrix Ap = SimpleMatrix.identity(n);
        for (int k = 0; k < n; k++) {
            if (k > 0) Ap = Ap.mult(A);
            C.insertIntoThis(0, k * m, Ap.mult(B));
        }
        return C;
    }

    static int numericalRank(SimpleMatrix M, double tol) {
        SimpleSVD<SimpleMatrix> svd = M.svd();
        int r = 0;
        for (int i = 0; i < svd.getSingularValues().length; i++) {
            if (svd.getSingularValues()[i] > tol) r++;
        }
        return r;
    }

    static SimpleMatrix nullspace(SimpleMatrix M, double tol) {
        SimpleSVD<SimpleMatrix> svd = M.svd();
        int rank = 0;
        double[] s = svd.getSingularValues();
        for (double value : s) if (value > tol) rank++;
        SimpleMatrix V = svd.getV();
        return V.extractMatrix(0, V.numRows(), rank, V.numCols());
    }

    static SimpleMatrix eigenstructurePolePlacement(SimpleMatrix A, SimpleMatrix B, double[] poles) {
        int n = A.numRows();
        int m = B.numCols();
        SimpleMatrix V = new SimpleMatrix(n, n);
        SimpleMatrix F = new SimpleMatrix(m, n);
        Random rng = new Random(24);

        for (int i = 0; i < n; i++) {
            double lambda = poles[i];
            SimpleMatrix M = new SimpleMatrix(n, n + m);
            M.insertIntoThis(0, 0, A.minus(SimpleMatrix.identity(n).scale(lambda)));
            M.insertIntoThis(0, n, B.scale(-1.0));
            SimpleMatrix N = nullspace(M, 1e-9);
            if (N.numCols() == 0) throw new RuntimeException("Empty nullspace.");

            boolean accepted = false;
            for (int trial = 0; trial < 300; trial++) {
                SimpleMatrix q = new SimpleMatrix(N.numCols(), 1);
                for (int j = 0; j < q.numRows(); j++) {
                    q.set(j, 0, trial == 0 ? 1.0 : rng.nextGaussian());
                }
                SimpleMatrix s = N.mult(q);
                SimpleMatrix v = s.extractMatrix(0, n, 0, 1);
                SimpleMatrix f = s.extractMatrix(n, n + m, 0, 1);

                SimpleMatrix Vtrial = V.copy();
                Vtrial.insertIntoThis(0, i, v);
                if (numericalRank(Vtrial.extractMatrix(0, n, 0, i + 1), 1e-8) == i + 1) {
                    V.insertIntoThis(0, i, v);
                    F.insertIntoThis(0, i, f);
                    accepted = true;
                    break;
                }
            }
            if (!accepted) throw new RuntimeException("Could not select independent eigenvector.");
        }
        return F.mult(V.invert());
    }

    public static void main(String[] args) {
        SimpleMatrix A = new SimpleMatrix(new double[][]{
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1},
            {-2, -5, -4, -1}
        });
        SimpleMatrix B = new SimpleMatrix(new double[][]{
            {0, 0},
            {1, 0},
            {0, 0},
            {0, 1}
        });
        double[] poles = {-1, -2, -3, -4};

        SimpleMatrix C = controllabilityMatrix(A, B);
        System.out.println("rank(C) = " + numericalRank(C, 1e-9) + " out of " + A.numRows());

        SimpleMatrix K = eigenstructurePolePlacement(A, B, poles);
        System.out.println("K =");
        K.print();

        SimpleEVD<SimpleMatrix> evd = A.minus(B.mult(K)).eig();
        System.out.println("Closed-loop eigenvalues:");
        for (int i = 0; i < evd.getNumberOfEigenvalues(); i++) {
            System.out.println(evd.getEigenvalue(i));
        }
    }
}
