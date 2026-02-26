import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

public class StabilityChecker {

    public static List<Complex[]> clusterRoots(Complex[] roots, double tol) {
        List<Complex[]> clusters = new ArrayList<>();
        boolean[] used = new boolean[roots.length];

        for (int i = 0; i < roots.length; ++i) {
            if (used[i]) continue;
            List<Complex> cluster = new ArrayList<>();
            cluster.add(roots[i]);
            used[i] = true;

            for (int j = i + 1; j < roots.length; ++j) {
                if (!used[j] && roots[j].subtract(roots[i]).abs() < tol) {
                    used[j] = true;
                    cluster.add(roots[j]);
                }
            }
            clusters.add(cluster.toArray(new Complex[0]));
        }
        return clusters;
    }

    public static String classify(double[] den, double tol) {
        PolynomialFunction p = new PolynomialFunction(den); // lowest order first
        LaguerreSolver solver = new LaguerreSolver();
        Complex[] roots = solver.solveAllComplex(den, 0.0);
        List<Complex[]> clusters = clusterRoots(roots, tol);

        boolean unstable = false;
        boolean marginal = false;

        for (Complex[] cluster : clusters) {
            Complex avg = Complex.ZERO;
            for (Complex c : cluster) {
                avg = avg.add(c);
            }
            avg = avg.divide(cluster.length);
            double re = avg.getReal();
            int mult = cluster.length;

            if (re > tol) unstable = true;
            else if (Math.abs(re) <= tol) {
                if (mult >= 2) unstable = true;
                else marginal = true;
            }
        }

        if (unstable) return "unstable";
        if (marginal) return "marginally stable";
        return "asymptotically stable";
    }

    public static void main(String[] args) {
        // Example: (s^2 + 1)^2 => coefficients in ascending powers of s
        double[] den1 = {1.0, 0.0, 2.0, 0.0, 1.0};
        System.out.println(classify(den1, 1e-6));
    }
}
