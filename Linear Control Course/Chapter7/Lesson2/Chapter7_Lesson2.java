// Example using Apache Commons Math
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;

public class StabilityCheck {

    public static String classifyEigenvalues(double[] real, double[] imag) {
        boolean hasPositive = false;
        boolean hasImagAxis = false;

        for (int i = 0; i < real.length; i++) {
            if (real[i] > 0.0) {
                hasPositive = true;
            }
            if (Math.abs(real[i]) < 1e-9 && Math.abs(imag[i]) > 1e-9) {
                hasImagAxis = true;
            }
        }

        if (hasPositive) return "unstable";
        if (hasImagAxis) return "marginally stable (oscillatory)";
        return "asymptotically stable";
    }

    public static void main(String[] args) {
        double J = 0.01;
        double b = 0.1;
        double k = 3.0;

        double[][] dataA = {
                {0.0,          1.0},
                {-k / J,  -b / J}
        };
        RealMatrix A = new Array2DRowRealMatrix(dataA);
        EigenDecomposition ed = new EigenDecomposition(A);

        int n = A.getRowDimension();
        double[] real = new double[n];
        double[] imag = new double[n];

        for (int i = 0; i < n; i++) {
            real[i] = ed.getRealEigenvalue(i);
            imag[i] = ed.getImagEigenvalue(i);
            System.out.println("lambda_" + i + " = " + real[i] + " + j" + imag[i]);
        }

        String stability = classifyEigenvalues(real, imag);
        System.out.println("Stability: " + stability);

        // This pattern can be embedded in Java-based robotic control loops
        // to verify that gain updates preserve closed-loop stability.
    }
}
