
import org.apache.commons.math3.linear.*;

public class DiscreteStabilityDemo {
    public static void main(String[] args) {
        double[][] adData = {
            {0.95, 0.01},
            {-0.2, 0.90}
        };
        RealMatrix Ad = MatrixUtils.createRealMatrix(adData);

        EigenDecomposition ed = new EigenDecomposition(Ad);
        double[] real = ed.getRealEigenvalues();
        double[] imag = ed.getImagEigenvalues();

        boolean stable = true;
        for (int i = 0; i < real.length; ++i) {
            double mag = Math.hypot(real[i], imag[i]);
            System.out.println("Pole " + i + ": " + real[i] + " + j" + imag[i]
                               + " |z| = " + mag);
            if (mag >= 1.0) stable = false;
        }
        System.out.println("Schur-stable? " + stable);
    }
}
