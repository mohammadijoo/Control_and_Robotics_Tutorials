/*
Chapter8_Lesson3.java

Computing Phi(t) = exp(A t) via eigen-decomposition in Java.

Dependency:
    Apache Commons Math 3.6.1 or later

Compile example:
    javac -cp commons-math3-3.6.1.jar Chapter8_Lesson3.java
Run example:
    java -cp .:commons-math3-3.6.1.jar Chapter8_Lesson3

Windows classpath separator:
    java -cp .;commons-math3-3.6.1.jar Chapter8_Lesson3

This example is designed for matrices with real eigenvalues and a complete real
eigenvector basis. Complex modal pairs require a real block-modal treatment or
a complex linear algebra library.
*/

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Chapter8_Lesson3 {

    public static RealMatrix phiViaEigendecomposition(RealMatrix A, double t) {
        EigenDecomposition eig = new EigenDecomposition(A);

        RealMatrix V = eig.getV();
        RealMatrix D = eig.getD();

        int n = A.getRowDimension();

        for (int i = 0; i < n; i++) {
            if (Math.abs(eig.getImagEigenvalue(i)) > 1e-12) {
                throw new IllegalArgumentException(
                    "This simple example expects real eigenvalues. Use a complex library for complex modes."
                );
            }
        }

        DecompositionSolver solver = new LUDecomposition(V).getSolver();
        if (!solver.isNonSingular()) {
            throw new IllegalArgumentException("A is not diagonalizable: V is singular.");
        }

        RealMatrix expDt = MatrixUtils.createRealMatrix(n, n);
        for (int i = 0; i < n; i++) {
            expDt.setEntry(i, i, Math.exp(D.getEntry(i, i) * t));
        }

        RealMatrix Vinv = solver.getInverse();
        return V.multiply(expDt).multiply(Vinv);
    }

    public static void printMatrix(String title, RealMatrix M) {
        System.out.println(title);
        for (int i = 0; i < M.getRowDimension(); i++) {
            for (int j = 0; j < M.getColumnDimension(); j++) {
                System.out.printf("%12.6f ", M.getEntry(i, j));
            }
            System.out.println();
        }
        System.out.println();
    }

    public static void main(String[] args) {
        double[][] data = {
            {-1.0,  2.0,  0.0},
            { 0.0, -2.0,  0.0},
            { 0.0,  0.0, -0.5}
        };

        RealMatrix A = new Array2DRowRealMatrix(data);
        double t = 2.0;

        RealMatrix Phi = phiViaEigendecomposition(A, t);

        printMatrix("A =", A);
        printMatrix("Phi(t) via eigen-decomposition =", Phi);

        RealVector x0 = MatrixUtils.createRealVector(new double[] {1.0, -1.0, 2.0});
        RealVector xt = Phi.operate(x0);

        System.out.println("x(t) = Phi(t) x0 =");
        for (int i = 0; i < xt.getDimension(); i++) {
            System.out.printf("%12.6f%n", xt.getEntry(i));
        }
    }
}
