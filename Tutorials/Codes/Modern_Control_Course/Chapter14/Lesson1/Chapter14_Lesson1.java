/*
Chapter14_Lesson1.java

Kalman observability matrix and rank condition using EJML.

Maven dependency example:
  <dependency>
    <groupId>org.ejml</groupId>
    <artifactId>ejml-simple</artifactId>
    <version>0.43.1</version>
  </dependency>
*/

import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

public class Chapter14_Lesson1 {

    public static SimpleMatrix observabilityMatrix(SimpleMatrix A, SimpleMatrix C) {
        int n = A.numRows();
        int p = C.numRows();

        SimpleMatrix O = new SimpleMatrix(p * n, n);
        SimpleMatrix Ak = SimpleMatrix.identity(n);

        for (int k = 0; k < n; k++) {
            SimpleMatrix block = C.mult(Ak);
            O.insertIntoThis(k * p, 0, block);
            Ak = Ak.mult(A);
        }
        return O;
    }

    public static int numericalRank(SimpleMatrix M) {
        SimpleSVD<SimpleMatrix> svd = M.svd();
        double[] s = svd.getSingularValues();

        double maxSingular = s.length > 0 ? s[0] : 0.0;
        double tol = Math.max(M.numRows(), M.numCols()) * Math.ulp(1.0) * maxSingular;

        int rank = 0;
        for (double value : s) {
            if (value > tol) {
                rank++;
            }
        }
        return rank;
    }

    public static void analyzeObservability(SimpleMatrix A, SimpleMatrix C, String name) {
        SimpleMatrix O = observabilityMatrix(A, C);
        SimpleSVD<SimpleMatrix> svd = O.svd();
        int rank = numericalRank(O);

        System.out.println("\n" + name);
        System.out.println("-".repeat(name.length()));
        System.out.println("A =");
        A.print();
        System.out.println("C =");
        C.print();
        System.out.println("Observability matrix O_n =");
        O.print();
        System.out.print("Singular values = ");
        for (double value : svd.getSingularValues()) {
            System.out.printf("%.8f ", value);
        }
        System.out.println();
        System.out.println("rank(O_n) = " + rank + " of n = " + A.numRows());
        System.out.println("Observable? " + (rank == A.numRows()));
    }

    public static void main(String[] args) {
        SimpleMatrix A1 = new SimpleMatrix(new double[][] {
                {0.0, 1.0},
                {0.0, 0.0}
        });
        SimpleMatrix C1 = new SimpleMatrix(new double[][] {
                {1.0, 0.0}
        });
        analyzeObservability(A1, C1, "Example 1: observable double integrator");

        SimpleMatrix A2 = new SimpleMatrix(new double[][] {
                {0.0, 0.0},
                {0.0, -2.0}
        });
        SimpleMatrix C2 = new SimpleMatrix(new double[][] {
                {1.0, 0.0}
        });
        analyzeObservability(A2, C2, "Example 2: unobservable second mode");
    }
}
