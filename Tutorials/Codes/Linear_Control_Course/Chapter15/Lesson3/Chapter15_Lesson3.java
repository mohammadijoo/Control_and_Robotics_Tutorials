import org.apache.commons.math3.complex.Complex;

public class NyquistRelativeStability {

    // Example: G(s) = 1 / (s (s + 1) (s + 2)), L(s) = K G(s)
    public static Complex Lofjw(double omega, double K) {
        Complex j = new Complex(0.0, 1.0);
        Complex s = j.multiply(omega);
        Complex denom = s.multiply(s.add(1.0)).multiply(s.add(2.0));
        return new Complex(K, 0.0).divide(denom);
    }

    public static void main(String[] args) {
        double K = 2.0;
        double dMin = Double.POSITIVE_INFINITY;
        double omegaMin = 0.0;

        for (int k = 0; k <= 1000; ++k) {
            double omega = Math.pow(10.0, -2.0 + 4.0 * k / 1000.0);
            Complex L = Lofjw(omega, K);
            Complex z = L.add(1.0); // 1 + L(jw)
            double d = z.abs();
            if (d < dMin) {
                dMin = d;
                omegaMin = omega;
            }
        }

        System.out.println("Approx d_min = " + dMin + " at omega = " + omegaMin);

        // In a robotics codebase, this can be used to check stability of joint controllers
        // when gains are auto-tuned or adapted online.
    }
}
