import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import java.util.Arrays;

public class RootLocusTools {

    public static PolynomialFunction polyFromRoots(Complex[] roots) {
        // Monic polynomial from roots.
        PolynomialFunction p = new PolynomialFunction(new double[]{1.0});
        for (Complex r : roots) {
            // Multiply by (s - r)
            PolynomialFunction q =
                new PolynomialFunction(new double[]{-r.getReal(), 1.0});
            // Imag part ignored for coefficients; complex roots should appear in conjugate pairs
            p = p.multiply(q);
        }
        return p;
    }

    public static PolynomialFunction polyDeriv(PolynomialFunction p) {
        return p.polynomialDerivative();
    }

    public static Complex[] breakawayCandidates(Complex[] poles, Complex[] zeros) {
        PolynomialFunction D = polyFromRoots(poles);
        PolynomialFunction N = zeros.length == 0
            ? new PolynomialFunction(new double[]{1.0})
            : polyFromRoots(zeros);

        PolynomialFunction Dp = polyDeriv(D);
        PolynomialFunction Np = polyDeriv(N);

        // P(s) = N(s) D'(s) - D(s) N'(s)
        PolynomialFunction NDp = N.multiply(Dp);
        PolynomialFunction DNp = D.multiply(Np);
        PolynomialFunction P = NDp.subtract(DNp);

        LaguerreSolver solver = new LaguerreSolver();
        Complex[] roots = solver.solveAllComplex(P.getCoefficients(), 0.0);

        return roots;
    }

    public static void main(String[] args) {
        Complex[] poles = new Complex[] {
            new Complex(-2.0, 0.0),
            new Complex(-5.0, 0.0),
            new Complex(-20.0, 0.0)
        };
        Complex[] zeros = new Complex[] {}; // none

        Complex[] cand = breakawayCandidates(poles, zeros);
        System.out.println("Breakaway / break-in candidates:");
        Arrays.stream(cand).forEach(c -> {
            if (Math.abs(c.getImaginary()) < 1e-6) {
                System.out.println("  s = " + c.getReal());
            }
        });
    }
}
