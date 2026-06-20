// Chapter13_Lesson3.java
// Detectability and Stable Unobservable Modes
//
// Dependency: Apache Commons Math 3
// Compile example:
//   javac -cp commons-math3-3.6.1.jar Chapter13_Lesson3.java
// Run example:
//   java -cp .:commons-math3-3.6.1.jar Chapter13_Lesson3
//
// On Windows PowerShell, use:
//   java -cp ".;commons-math3-3.6.1.jar" Chapter13_Lesson3

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

public class Chapter13_Lesson3 {
    public static RealMatrix observabilityMatrix(RealMatrix A, RealMatrix C) {
        int n = A.getRowDimension();
        int p = C.getRowDimension();

        RealMatrix O = new Array2DRowRealMatrix(p * n, n);
        RealMatrix Ak = MatrixUtils.createRealIdentityMatrix(n);

        for (int k = 0; k < n; k++) {
            RealMatrix block = C.multiply(Ak);
            O.setSubMatrix(block.getData(), k * p, 0);
            Ak = Ak.multiply(A);
        }
        return O;
    }

    public static RealMatrix nullspace(RealMatrix M, double tol) {
        SingularValueDecomposition svd = new SingularValueDecomposition(M);
        double[] s = svd.getSingularValues();
        int n = M.getColumnDimension();

        int rank = 0;
        double scale = s.length == 0 ? 0.0 : s[0];
        for (double value : s) {
            if (value > tol * Math.max(M.getRowDimension(), M.getColumnDimension()) * scale) {
                rank++;
            }
        }

        RealMatrix V = svd.getV();
        int nullity = n - rank;
        if (nullity == 0) {
            return new Array2DRowRealMatrix(n, 0);
        }

        RealMatrix N = new Array2DRowRealMatrix(n, nullity);
        for (int j = 0; j < nullity; j++) {
            N.setColumnVector(j, V.getColumnVector(rank + j));
        }
        return N;
    }

    public static double[][] unobservableModes(RealMatrix A, RealMatrix C, double tol) {
        RealMatrix O = observabilityMatrix(A, C);
        RealMatrix N = nullspace(O, tol);

        if (N.getColumnDimension() == 0) {
            return new double[][] { new double[0], new double[0] };
        }

        RealMatrix Ahidden = N.transpose().multiply(A).multiply(N);
        EigenDecomposition eig = new EigenDecomposition(Ahidden);
        return new double[][] { eig.getRealEigenvalues(), eig.getImagEigenvalues() };
    }

    public static boolean isDetectable(RealMatrix A, RealMatrix C,
                                       String system, double tol) {
        double[][] hidden = unobservableModes(A, C, tol);
        double[] re = hidden[0];
        double[] im = hidden[1];

        if (re.length == 0) {
            return true;
        }

        if (system.equalsIgnoreCase("continuous")) {
            for (double value : re) {
                if (value >= -tol) {
                    return false;
                }
            }
            return true;
        }

        if (system.equalsIgnoreCase("discrete")) {
            for (int i = 0; i < re.length; i++) {
                double modulus = Math.hypot(re[i], im[i]);
                if (modulus >= 1.0 - tol) {
                    return false;
                }
            }
            return true;
        }

        throw new IllegalArgumentException("system must be continuous or discrete");
    }

    public static void report(double[][] Adata, double[][] Cdata,
                              String name, String system) {
        RealMatrix A = new Array2DRowRealMatrix(Adata);
        RealMatrix C = new Array2DRowRealMatrix(Cdata);
        RealMatrix O = observabilityMatrix(A, C);

        SingularValueDecomposition svd = new SingularValueDecomposition(O);
        int rankO = svd.getRank();

        double[][] hidden = unobservableModes(A, C, 1e-10);
        double[] re = hidden[0];
        double[] im = hidden[1];

        System.out.println("\n" + name);
        System.out.println("-".repeat(name.length()));
        System.out.println("rank(O) = " + rankO + " of n = " + A.getRowDimension());

        System.out.print("unobservable modes = ");
        for (int i = 0; i < re.length; i++) {
            System.out.print("(" + re[i] + " + " + im[i] + "i) ");
        }
        System.out.println();

        System.out.println("detectable = " + isDetectable(A, C, system, 1e-9));
    }

    public static void main(String[] args) {
        double[][] A1 = {
            {-1.0, 0.0, 0.0},
            { 0.0, 2.0, 0.0},
            { 0.0, 0.0, -0.5}
        };
        double[][] C1 = { {0.0, 1.0, 0.0} };
        report(A1, C1, "Continuous-time: detectable but not observable", "continuous");

        double[][] A2 = {
            { 1.0, 0.0, 0.0},
            { 0.0, -2.0, 0.0},
            { 0.0, 0.0, -0.5}
        };
        double[][] C2 = { {0.0, 1.0, 0.0} };
        report(A2, C2, "Continuous-time: not detectable", "continuous");

        double[][] A3 = {
            {0.3, 0.0, 0.0},
            {0.0, 1.2, 0.0},
            {0.0, 0.0, -0.7}
        };
        double[][] C3 = { {0.0, 1.0, 0.0} };
        report(A3, C3, "Discrete-time: detectable but not observable", "discrete");
    }
}
