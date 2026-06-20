// Chapter24_Lesson4.java
// Partial Pole Placement and Restricted Eigenstructure
// Dependency: Apache Commons Math 3
// Compile example:
// javac -cp commons-math3-3.6.1.jar Chapter24_Lesson4.java
// java  -cp .:commons-math3-3.6.1.jar Chapter24_Lesson4

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.ArrayRealVector;

public class Chapter24_Lesson4 {
    static void printMatrix(String name, RealMatrix M) {
        System.out.println(name + " =");
        for (int i = 0; i < M.getRowDimension(); i++) {
            for (int j = 0; j < M.getColumnDimension(); j++) {
                System.out.printf("%12.6f ", M.getEntry(i, j));
            }
            System.out.println();
        }
        System.out.println();
    }

    static RealMatrix hstack(RealMatrix left, RealMatrix right) {
        int rows = left.getRowDimension();
        int cols = left.getColumnDimension() + right.getColumnDimension();
        RealMatrix out = MatrixUtils.createRealMatrix(rows, cols);
        out.setSubMatrix(left.getData(), 0, 0);
        out.setSubMatrix(right.getData(), 0, left.getColumnDimension());
        return out;
    }

    public static void main(String[] args) {
        int n = 4;
        double[][] Adata = {
            {0.2, 0.0, 0.0, 0.0},
            {0.0, 0.6, 0.0, 0.0},
            {0.0, 0.0, -0.5, 0.0},
            {0.0, 0.0, 0.0, -1.0}
        };
        double[][] Bdata = {
            {1.0, 0.0},
            {0.3, 1.0},
            {0.2, 0.4},
            {0.1, 0.2}
        };

        RealMatrix A = new Array2DRowRealMatrix(Adata);
        RealMatrix B = new Array2DRowRealMatrix(Bdata);
        double[] lambdas = {-2.0, -3.0};
        double[][] Gdata = {
            {1.0, 0.0},
            {0.0, 1.0}
        };
        RealMatrix G = new Array2DRowRealMatrix(Gdata);

        RealMatrix V = MatrixUtils.createRealMatrix(n, 2);
        for (int i = 0; i < 2; i++) {
            RealMatrix M = A.subtract(MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(lambdas[i]));
            RealVector g = G.getColumnVector(i);
            RealVector v = MatrixUtils.inverse(M).operate(B.operate(g));
            V.setColumnVector(i, v);
        }

        RealMatrix Nkeep = MatrixUtils.createRealMatrix(n, 2);
        Nkeep.setEntry(2, 0, 1.0);
        Nkeep.setEntry(3, 1, 1.0);

        RealMatrix X = hstack(V, Nkeep);
        RealMatrix zeros = MatrixUtils.createRealMatrix(2, 2);
        RealMatrix Y = hstack(G, zeros);
        RealMatrix K = Y.multiply(MatrixUtils.inverse(X));
        RealMatrix Acl = A.subtract(B.multiply(K));

        printMatrix("K", K);

        EigenDecomposition eig = new EigenDecomposition(Acl);
        System.out.println("Closed-loop eigenvalues:");
        for (double val : eig.getRealEigenvalues()) {
            System.out.printf("%12.6f ", val);
        }
        System.out.println("\n");

        RealMatrix Lambda = MatrixUtils.createRealDiagonalMatrix(lambdas);
        double assignedResidual = Acl.multiply(V).subtract(V.multiply(Lambda)).getFrobeniusNorm();
        double keepResidual = K.multiply(Nkeep).getFrobeniusNorm();
        System.out.printf("Assigned-mode residual = %.6e\n", assignedResidual);
        System.out.printf("Preservation residual ||K Nkeep|| = %.6e\n\n", keepResidual);

        // Restricted eigenstructure: require x3 = x4 for lambda = -2.
        RealMatrix H = new Array2DRowRealMatrix(new double[][] {{0.0, 0.0, 1.0, -1.0}});
        double lam = -2.0;
        RealMatrix M = MatrixUtils.inverse(
            A.subtract(MatrixUtils.createRealIdentityMatrix(n).scalarMultiply(lam))
        ).multiply(B);
        RealMatrix S = H.multiply(M);
        double a = S.getEntry(0, 0);
        double b = S.getEntry(0, 1);
        RealVector grest = new ArrayRealVector(new double[] {b, -a});
        RealVector vrest = M.operate(grest);

        printMatrix("S", S);
        System.out.println("Restricted g = " + grest);
        System.out.println("Restricted v = " + vrest);
        System.out.println("H v = " + H.operate(vrest));
    }
}
