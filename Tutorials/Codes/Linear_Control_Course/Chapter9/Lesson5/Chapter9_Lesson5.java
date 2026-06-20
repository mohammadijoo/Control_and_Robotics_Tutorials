import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;

public class RootLocusCubic {
    // P(s, K) = s^3 + 7 s^2 + 10 s + K
    public static Complex[] rootsForGain(double K) {
        // Coefficients in descending powers: [1, 7, 10, K]
        double[] coeffs = new double[] {1.0, 7.0, 10.0, K};
        LaguerreSolver solver = new LaguerreSolver();
        // Initial guess (not used by solveAllComplex internally)
        Complex[] roots = solver.solveAllComplex(coeffs, 0.0);
        return roots;
    }

    public static void main(String[] args) {
        double Kmin = 0.0, Kmax = 80.0;
        int numK = 20;
        for (int j = 0; j <= numK; ++j) {
            double K = Kmin + (Kmax - Kmin) * j / (double) numK;
            Complex[] roots = rootsForGain(K);
            System.out.print("K = " + K + " poles: ");
            for (Complex r : roots) {
                System.out.print(r + " ");
            }
            System.out.println();
        }
    }
}
