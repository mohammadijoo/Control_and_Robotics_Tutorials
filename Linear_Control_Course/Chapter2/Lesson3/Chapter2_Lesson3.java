public class LaplaceNumeric {

    // Interface for a scalar function of time
    @FunctionalInterface
    public interface TimeFunction {
        double value(double t);
    }

    // Trapezoidal-rule approximation of Laplace transform at real s > 0
    public static double laplaceNumeric(TimeFunction f, double s,
                                        double tMax, int N) {
        double h = tMax / (double) N;
        double sum = 0.0;
        for (int k = 0; k <= N; ++k) {
            double t = k * h;
            double w = (k == 0 || k == N) ? 0.5 : 1.0;
            sum += w * f.value(t) * Math.exp(-s * t);
        }
        return h * sum;
    }

    public static void main(String[] args) {
        // Example: f(t) = u(t) (unit step), analytic F(s) = 1/s
        TimeFunction step = (double t) -> (t >= 0.0 ? 1.0 : 0.0);

        double s = 3.0;
        double tMax = 20.0;
        int N = 20000;

        double numeric = laplaceNumeric(step, s, tMax, N);
        double exact = 1.0 / s;

        System.out.println("Approximate F(" + s + ") = " + numeric);
        System.out.println("Exact       F(" + s + ") = " + exact);
    }
}
