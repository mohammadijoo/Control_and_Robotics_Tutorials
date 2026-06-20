/*
Chapter17_Lesson3.java
Diagonal modal form for a continuous-time LTI system with distinct real eigenvalues.

Required library:
    Apache Commons Math 3.x
Compile example:
    javac -cp commons-math3-3.6.1.jar Chapter17_Lesson3.java
Run example:
    java -cp .:commons-math3-3.6.1.jar Chapter17_Lesson3
*/

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Chapter17_Lesson3 {
    public static void main(String[] args) {
        double[][] aData = {
            {0.0, 1.0, 0.0},
            {-2.0, -3.0, 0.0},
            {0.5, 0.0, -4.0}
        };
        double[][] bData = {
            {0.0},
            {1.0},
            {0.0}
        };
        double[][] cData = {
            {1.0, 0.0, 1.0}
        };

        RealMatrix A = new Array2DRowRealMatrix(aData);
        RealMatrix B = new Array2DRowRealMatrix(bData);
        RealMatrix C = new Array2DRowRealMatrix(cData);

        EigenDecomposition ed = new EigenDecomposition(A);
        RealMatrix V = ed.getV();
        RealMatrix Vinv = new LUDecomposition(V).getSolver().getInverse();
        RealMatrix Lambda = Vinv.multiply(A).multiply(V);
        RealMatrix Bm = Vinv.multiply(B);
        RealMatrix Cm = C.multiply(V);

        System.out.println("Eigenvalues:");
        for (int i = 0; i < A.getRowDimension(); i++) {
            System.out.printf("lambda_%d = %.8f%n", i + 1, ed.getRealEigenvalue(i));
        }

        printMatrix("Modal A matrix Lambda = V^{-1} A V", Lambda);
        printMatrix("Modal input matrix Bm = V^{-1} B", Bm);
        printMatrix("Modal output matrix Cm = C V", Cm);

        double[] x0Data = {1.0, 0.0, -0.5};
        RealVector x0 = new Array2DRowRealMatrix(new double[][]{{1.0}, {0.0}, {-0.5}}).getColumnVector(0);
        RealVector z0 = Vinv.operate(x0);

        System.out.println("Zero-input modal response y(t):");
        for (int step = 0; step <= 5; step++) {
            double t = step;
            double[][] expData = new double[3][3];
            for (int i = 0; i < 3; i++) {
                expData[i][i] = Math.exp(ed.getRealEigenvalue(i) * t);
            }
            RealMatrix expLambda = new Array2DRowRealMatrix(expData);
            RealVector zt = expLambda.operate(z0);
            RealVector xt = V.operate(zt);
            double y = C.operate(xt).getEntry(0);
            System.out.printf("t=%.1f, y=%.8f%n", t, y);
        }
    }

    private static void printMatrix(String title, RealMatrix M) {
        System.out.println("\n" + title + ":");
        for (int i = 0; i < M.getRowDimension(); i++) {
            for (int j = 0; j < M.getColumnDimension(); j++) {
                System.out.printf("%12.6f ", M.getEntry(i, j));
            }
            System.out.println();
        }
    }
}
