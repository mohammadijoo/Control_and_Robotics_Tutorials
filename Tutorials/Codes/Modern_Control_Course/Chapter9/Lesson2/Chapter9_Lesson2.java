// Chapter9_Lesson2.java
// Eigenvalue-based stability criteria using Apache Commons Math.
//
// Compile/run example:
//   javac -cp commons-math3-3.6.1.jar Chapter9_Lesson2.java
//   java  -cp .;commons-math3-3.6.1.jar Chapter9_Lesson2
//
// On Linux/macOS use ":" instead of ";" in the classpath.

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

public class Chapter9_Lesson2 {
    static final double TOL = 1e-10;

    public static String classifyContinuous(double[][] data) {
        RealMatrix A = new Array2DRowRealMatrix(data);
        EigenDecomposition eig = new EigenDecomposition(A);

        double maxReal = -Double.MAX_VALUE;
        for (int i = 0; i < A.getRowDimension(); i++) {
            maxReal = Math.max(maxReal, eig.getRealEigenvalue(i));
        }

        if (maxReal < -TOL) return "asymptotically stable";
        if (maxReal > TOL) return "unstable";
        return "borderline: check semisimplicity of imaginary-axis eigenvalues";
    }

    public static String classifyDiscrete(double[][] data) {
        RealMatrix A = new Array2DRowRealMatrix(data);
        EigenDecomposition eig = new EigenDecomposition(A);

        double spectralRadius = 0.0;
        for (int i = 0; i < A.getRowDimension(); i++) {
            double real = eig.getRealEigenvalue(i);
            double imag = eig.getImagEigenvalue(i);
            double magnitude = Math.hypot(real, imag);
            spectralRadius = Math.max(spectralRadius, magnitude);
        }

        if (spectralRadius < 1.0 - TOL) return "asymptotically stable";
        if (spectralRadius > 1.0 + TOL) return "unstable";
        return "borderline: check semisimplicity of unit-circle eigenvalues";
    }

    public static void printReport(String name, double[][] data, boolean continuous) {
        RealMatrix A = new Array2DRowRealMatrix(data);
        EigenDecomposition eig = new EigenDecomposition(A);

        System.out.println("====================================================");
        System.out.println(name);
        System.out.println("Eigenvalues:");
        for (int i = 0; i < A.getRowDimension(); i++) {
            System.out.printf("  %.8f %+ .8fi%n",
                    eig.getRealEigenvalue(i), eig.getImagEigenvalue(i));
        }

        if (continuous) {
            System.out.println("Classification: " + classifyContinuous(data));
        } else {
            System.out.println("Classification: " + classifyDiscrete(data));
        }
    }

    public static void main(String[] args) {
        double[][] A1 = {
            {-1.0, 0.0},
            { 0.0,-2.0}
        };

        double[][] A2 = {
            {0.0, -1.0},
            {1.0,  0.0}
        };

        double[][] A3 = {
            {0.0, 1.0},
            {0.0, 0.0}
        };

        double[][] Ad1 = {
            {0.8, 0.0},
            {0.0, 0.5}
        };

        printReport("Continuous asymptotically stable example", A1, true);
        printReport("Continuous oscillatory borderline example", A2, true);
        printReport("Continuous defective zero-eigenvalue example", A3, true);
        printReport("Discrete asymptotically stable example", Ad1, false);
    }
}
