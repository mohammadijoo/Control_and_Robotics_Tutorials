import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;

public class JointPoles {
    public static void main(String[] args) {
        double J = 0.01;
        double B = 0.02;
        double K = 1.0;

        // Denominator: J s^2 + B s + K
        double[] coeffs = {K, B, J}; // constant term first
        PolynomialFunction den = new PolynomialFunction(coeffs);

        LaguerreSolver solver = new LaguerreSolver();
        // For quadratics, we know there are two roots; use complex root-finding
        org.apache.commons.math3.complex.Complex[] roots = solver.solveAllComplex(coeffs, 0.0);

        for (org.apache.commons.math3.complex.Complex r : roots) {
            System.out.println("Pole: " + r);
        }
    }
}
