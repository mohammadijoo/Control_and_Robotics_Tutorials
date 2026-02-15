import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

public class StabilityChecker {

    public static void main(String[] args) {
        // Example: G(s) = 1 / (s^2 + 3 s + 2)
        // Coefficients in descending powers: a_n, ..., a_0
        double[] den = {1.0, 3.0, 2.0};

        LaguerreSolver solver = new LaguerreSolver();
        Complex[] roots = solver.solveAllComplex(den, 0.0);

        boolean asymptoticallyStable = true;
        for (Complex r : roots) {
            double realPart = r.getReal();
            System.out.println("pole = " + r);
            if (realPart >= 0.0) {
                asymptoticallyStable = false;
            }
        }

        if (asymptoticallyStable) {
            System.out.println("System is asymptotically stable.");
        } else {
            System.out.println("System is NOT asymptotically stable.");
        }
    }
}
