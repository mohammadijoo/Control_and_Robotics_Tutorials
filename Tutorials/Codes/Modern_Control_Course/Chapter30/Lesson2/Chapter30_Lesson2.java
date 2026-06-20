/*
Chapter30_Lesson2.java
Coordinate scaling and numerical conditioning for state-space systems.

Dependencies:
    EJML dense row module, for example:
    ejml-simple, ejml-ddense, ejml-core

Compile example:
    javac -cp ".;ejml-simple.jar;ejml-ddense.jar;ejml-core.jar" Chapter30_Lesson2.java
Run example:
    java  -cp ".;ejml-simple.jar;ejml-ddense.jar;ejml-core.jar" Chapter30_Lesson2
*/

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

public class Chapter30_Lesson2 {
    static SimpleMatrix controllabilityMatrix(SimpleMatrix A, SimpleMatrix B) {
        int n = A.numRows();
        int m = B.numCols();
        SimpleMatrix Mc = new SimpleMatrix(n, n * m);
        SimpleMatrix Apow = SimpleMatrix.identity(n);
        for (int k = 0; k < n; k++) {
            Mc.insertIntoThis(0, k * m, Apow.mult(B));
            Apow = Apow.mult(A);
        }
        return Mc;
    }

    static SimpleMatrix observabilityMatrix(SimpleMatrix A, SimpleMatrix C) {
        int n = A.numRows();
        int p = C.numRows();
        SimpleMatrix Mo = new SimpleMatrix(n * p, n);
        SimpleMatrix Apow = SimpleMatrix.identity(n);
        for (int k = 0; k < n; k++) {
            Mo.insertIntoThis(k * p, 0, C.mult(Apow));
            Apow = Apow.mult(A);
        }
        return Mo;
    }

    static double conditionNumber2(SimpleMatrix M) {
        SimpleSVD<SimpleMatrix> svd = M.svd();
        double[] s = svd.getSingularValues();
        double smax = 0.0;
        double smin = Double.POSITIVE_INFINITY;
        for (double value : s) {
            smax = Math.max(smax, value);
            smin = Math.min(smin, value);
        }
        return smax / smin;
    }

    static SimpleMatrix diagonal(double[] values) {
        SimpleMatrix D = new SimpleMatrix(values.length, values.length);
        for (int i = 0; i < values.length; i++) {
            D.set(i, i, values[i]);
        }
        return D;
    }

    public static void main(String[] args) {
        SimpleMatrix A = new SimpleMatrix(new double[][] {
            {0.0,       1.0,       0.0},
            {-2.0e3,   -5.0e1,    8.0e4},
            {0.0,      -2.0e-2,  -4.0e3}
        });

        SimpleMatrix B = new SimpleMatrix(new double[][] {
            {0.0},
            {0.0},
            {2.0e3}
        });

        SimpleMatrix C = new SimpleMatrix(new double[][] {
            {1.0, 0.0, 0.0}
        });

        double[] xNom = {1.0e-3, 1.0e-1, 1.0e1};
        double[] xNomInv = {1.0 / xNom[0], 1.0 / xNom[1], 1.0 / xNom[2]};

        SimpleMatrix S = diagonal(xNom);
        SimpleMatrix Sinv = diagonal(xNomInv);

        SimpleMatrix Az = Sinv.mult(A).mult(S);
        SimpleMatrix Bz = Sinv.mult(B);
        SimpleMatrix Cz = C.mult(S);

        SimpleEVD<SimpleMatrix> evdA = A.eig();
        SimpleEVD<SimpleMatrix> evdAz = Az.eig();

        System.out.println("Eigenvalues of A:");
        for (int i = 0; i < evdA.getNumberOfEigenvalues(); i++) {
            System.out.println(evdA.getEigenvalue(i));
        }

        System.out.println("\nEigenvalues of Az:");
        for (int i = 0; i < evdAz.getNumberOfEigenvalues(); i++) {
            System.out.println(evdAz.getEigenvalue(i));
        }

        SimpleMatrix Mc = controllabilityMatrix(A, B);
        SimpleMatrix Mcz = controllabilityMatrix(Az, Bz);
        SimpleMatrix Mo = observabilityMatrix(A, C);
        SimpleMatrix Moz = observabilityMatrix(Az, Cz);

        System.out.printf("%ncond(A)             = %.6e%n", conditionNumber2(A));
        System.out.printf("cond(Az)            = %.6e%n", conditionNumber2(Az));
        System.out.printf("cond(Mc)            = %.6e%n", conditionNumber2(Mc));
        System.out.printf("cond(Mc scaled)     = %.6e%n", conditionNumber2(Mcz));
        System.out.printf("cond(Mo)            = %.6e%n", conditionNumber2(Mo));
        System.out.printf("cond(Mo scaled)     = %.6e%n", conditionNumber2(Moz));

        System.out.println("\nAz:");
        Az.print();
        System.out.println("Bz:");
        Bz.print();
        System.out.println("Cz:");
        Cz.print();
    }
}
