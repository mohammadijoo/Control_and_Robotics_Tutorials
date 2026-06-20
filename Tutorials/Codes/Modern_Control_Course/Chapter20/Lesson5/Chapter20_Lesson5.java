// Chapter20_Lesson5.java
/*
Numerical realization diagnostics using Apache Commons Math.

Dependency:
    commons-math3-3.6.1.jar

Compile:
    javac -cp commons-math3-3.6.1.jar Chapter20_Lesson5.java

Run:
    java -cp .:commons-math3-3.6.1.jar Chapter20_Lesson5
On Windows use:
    java -cp .;commons-math3-3.6.1.jar Chapter20_Lesson5
*/

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

public class Chapter20_Lesson5 {
    static RealMatrix multiplyPower(RealMatrix A, int k) {
        RealMatrix P = MatrixUtils.createRealIdentityMatrix(A.getRowDimension());
        for (int i = 0; i < k; i++) {
            P = P.multiply(A);
        }
        return P;
    }

    static RealMatrix horizontalStack(RealMatrix[] blocks) {
        int rows = blocks[0].getRowDimension();
        int cols = 0;
        for (RealMatrix b : blocks) {
            cols += b.getColumnDimension();
        }
        RealMatrix out = MatrixUtils.createRealMatrix(rows, cols);
        int c0 = 0;
        for (RealMatrix b : blocks) {
            out.setSubMatrix(b.getData(), 0, c0);
            c0 += b.getColumnDimension();
        }
        return out;
    }

    static RealMatrix verticalStack(RealMatrix[] blocks) {
        int cols = blocks[0].getColumnDimension();
        int rows = 0;
        for (RealMatrix b : blocks) {
            rows += b.getRowDimension();
        }
        RealMatrix out = MatrixUtils.createRealMatrix(rows, cols);
        int r0 = 0;
        for (RealMatrix b : blocks) {
            out.setSubMatrix(b.getData(), r0, 0);
            r0 += b.getRowDimension();
        }
        return out;
    }

    static RealMatrix ctrb(RealMatrix A, RealMatrix B) {
        int n = A.getRowDimension();
        RealMatrix[] blocks = new RealMatrix[n];
        RealMatrix Ap = MatrixUtils.createRealIdentityMatrix(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = Ap.multiply(B);
            Ap = Ap.multiply(A);
        }
        return horizontalStack(blocks);
    }

    static RealMatrix obsv(RealMatrix A, RealMatrix C) {
        int n = A.getRowDimension();
        RealMatrix[] blocks = new RealMatrix[n];
        RealMatrix Ap = MatrixUtils.createRealIdentityMatrix(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = C.multiply(Ap);
            Ap = Ap.multiply(A);
        }
        return verticalStack(blocks);
    }

    static int numericalRank(RealMatrix M) {
        SingularValueDecomposition svd = new SingularValueDecomposition(M);
        double[] s = svd.getSingularValues();
        if (s.length == 0) return 0;
        double eps = Math.ulp(1.0);
        double tol = Math.max(M.getRowDimension(), M.getColumnDimension()) * eps * s[0];
        int rank = 0;
        for (double value : s) {
            if (value > tol) rank++;
        }
        return rank;
    }

    static void printSingularValues(String name, RealMatrix M) {
        SingularValueDecomposition svd = new SingularValueDecomposition(M);
        System.out.println(name + " singular values:");
        for (double s : svd.getSingularValues()) {
            System.out.printf("% .6e ", s);
        }
        System.out.println("\n");
    }

    public static void main(String[] args) {
        double[][] Adata = {
            {-0.20,  0.05,  0.00,  0.00},
            { 0.00, -1.00,  0.10,  0.00},
            { 0.00,  0.00, -8.00,  0.20},
            { 0.00,  0.00,  0.00, -20.0}
        };
        double[][] Bdata = {{1.0}, {0.4}, {0.05}, {0.01}};
        double[][] Cdata = {{1.0, 0.3, 0.02, 0.005}};

        RealMatrix A = MatrixUtils.createRealMatrix(Adata);
        RealMatrix B = MatrixUtils.createRealMatrix(Bdata);
        RealMatrix C = MatrixUtils.createRealMatrix(Cdata);

        RealMatrix Rc = ctrb(A, B);
        RealMatrix Ro = obsv(A, C);

        printSingularValues("Controllability matrix", Rc);
        printSingularValues("Observability matrix", Ro);

        System.out.println("Numerical controllability rank = " + numericalRank(Rc));
        System.out.println("Numerical observability rank = " + numericalRank(Ro));

        SingularValueDecomposition svdC = new SingularValueDecomposition(Rc);
        double[] s = svdC.getSingularValues();
        double conditionEstimate = s[0] / s[s.length - 1];

        System.out.printf("Estimated condition number of controllability matrix = %.6e%n",
                          conditionEstimate);

        System.out.println("\nLesson point:");
        System.out.println("Near-zero singular values indicate weakly reachable or weakly");
        System.out.println("observable directions. Such directions are not safely detected");
        System.out.println("by exact rank logic in finite precision arithmetic.");
    }
}
