import org.apache.commons.math3.complex.Complex;

public class NicholsSensitivity {

    private static final double J = 0.01;
    private static final double B = 0.1;
    private static final double Kp = 20.0;
    private static final double Kd = 1.0;

    static Complex G(double w) {
        Complex s = new Complex(0.0, w);
        return Complex.ONE.divide(
                s.multiply(s).multiply(J).add(s.multiply(B))
        );
    }

    static Complex C(double w) {
        Complex s = new Complex(0.0, w);
        return s.multiply(Kd).add(Kp);
    }

    public static void main(String[] args) {
        int N = 400;
        double logwMin = Math.log10(0.1);
        double logwMax = Math.log10(100.0);

        double Ms = 0.0;

        for (int k = 0; k < N; ++k) {
            double alpha = (double) k / (double) (N - 1);
            double logw = logwMin + alpha * (logwMax - logwMin);
            double w = Math.pow(10.0, logw);

            Complex L = C(w).multiply(G(w));
            Complex S = Complex.ONE.divide(Complex.ONE.add(L));
            double magS = S.abs();
            if (magS > Ms) {
                Ms = magS;
            }

            double phaseDeg = Math.toDegrees(L.getArgument());
            double magDb = 20.0 * Math.log10(L.abs());
            System.out.printf("%f %f%n", phaseDeg, magDb);
        }

        System.err.printf("Peak sensitivity Ms = %.3f%n", Ms);
    }
}
