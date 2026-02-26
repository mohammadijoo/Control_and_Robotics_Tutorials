
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import java.util.Random;

public class JointStabilityCheck {
    private static RealMatrix Acl(double J, double Kp, double Kd) {
        double[][] data = {
            {0.0, 1.0},
            {-Kp / J, -Kd / J}
        };
        return new Array2DRowRealMatrix(data);
    }

    public static void main(String[] args) {
        double J_nom = 0.5;
        double Kp = 50.0;
        double Kd = 5.0;

        RealMatrix A = Acl(J_nom, Kp, Kd);
        EigenDecomposition ed = new EigenDecomposition(A);
        double[] realParts = ed.getRealEigenvalues();
        double alpha = -1e9;
        for (double r : realParts) {
            if (r > alpha) alpha = r;
        }
        System.out.println("Nominal spectral abscissa: " + alpha);

        // Monte Carlo over inertia uncertainty
        Random rng = new Random(0);
        int numSamples = 200;
        int unstable = 0;
        double worstAlpha = -1e9;

        for (int k = 0; k < numSamples; ++k) {
            double delta = 0.3 * (2.0 * rng.nextDouble() - 1.0); // +-30%
            double J = J_nom * (1.0 + delta);
            RealMatrix Ak = Acl(J, Kp, Kd);
            EigenDecomposition edk = new EigenDecomposition(Ak);
            double[] rparts = edk.getRealEigenvalues();
            double alpha_k = -1e9;
            for (double r : rparts) {
                if (r > alpha_k) alpha_k = r;
            }
            worstAlpha = Math.max(worstAlpha, alpha_k);
            if (alpha_k >= 0.0) unstable++;
        }

        System.out.println("Worst spectral abscissa over samples: " + worstAlpha);
        System.out.println("Unstable samples: " + unstable);
    }
}
