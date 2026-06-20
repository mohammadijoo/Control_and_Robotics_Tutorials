/*
Chapter21_Lesson2.java
Computation of zeros for the special case where D is nonsingular.

Requires EJML:
    ejml-simple and ejml-ddense on the classpath.

For nonsingular square D, the zeros are the eigenvalues of
    A_z = A - B D^{-1} C.
For singular D or rectangular MIMO systems, use a Rosenbrock staircase / Kronecker
pencil implementation in a numerical linear algebra package.
*/

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.ejml.data.Complex_F64;

public class Chapter21_Lesson2 {
    public static SimpleMatrix zeroMatrixWhenDInvertible(SimpleMatrix A,
                                                         SimpleMatrix B,
                                                         SimpleMatrix C,
                                                         SimpleMatrix D) {
        if (D.numRows() != D.numCols()) {
            throw new IllegalArgumentException("D must be square.");
        }
        return A.minus(B.mult(D.invert()).mult(C));
    }

    public static void main(String[] args) {
        SimpleMatrix A = new SimpleMatrix(new double[][]{
            {0.0, 1.0},
            {-2.0, -3.0}
        });
        SimpleMatrix B = new SimpleMatrix(new double[][]{
            {0.0},
            {1.0}
        });
        SimpleMatrix C = new SimpleMatrix(new double[][]{
            {4.0, 1.0}
        });
        SimpleMatrix D = new SimpleMatrix(new double[][]{
            {1.0}
        });

        SimpleMatrix Az = zeroMatrixWhenDInvertible(A, B, C, D);
        SimpleEVD<SimpleMatrix> evd = Az.eig();

        System.out.println("A_z = A - B D^{-1} C:");
        Az.print();
        System.out.println("Transmission zeros for nonsingular D:");
        for (int i = 0; i < evd.getNumberOfEigenvalues(); i++) {
            Complex_F64 lambda = evd.getEigenvalue(i);
            System.out.println(lambda);
        }
    }
}
