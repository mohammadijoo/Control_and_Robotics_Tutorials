import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

public class RelativeStabilityJava {

    // Degree of stability for P(s) = s^3 + 5 s^2 + 6 s + K
    public static double degreeOfStability(double K) {
        double[] coeffs = {1.0, 5.0, 6.0, K};  // highest to lowest degree
        LaguerreSolver solver = new LaguerreSolver();
        Complex[] roots = solver.solveAllComplex(coeffs, 0.0);

        double alpha = Double.POSITIVE_INFINITY;
        for (Complex r : roots) {
            double realPart = r.getReal();
            if (realPart >= 0.0) {
                return 0.0; // unstable
            }
            alpha = Math.min(alpha, -realPart);
        }
        return alpha;
    }

    public static void main(String[] args) {
        double K = 10.0;
        double alpha = degreeOfStability(K);
        if (alpha > 0.0) {
            System.out.println("Stable, alpha = " + alpha);
        } else {
            System.out.println("Unstable");
        }
    }
}
