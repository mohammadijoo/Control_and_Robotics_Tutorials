import org.apache.commons.math3.complex.Complex;
import java.util.ArrayList;
import java.util.List;

public class NicholsRobustness {
    public static void main(String[] args) {
        double J = 0.02;
        double B = 0.1;
        double K = 1.0;

        double kp = 50.0;
        double Ti = 0.1;
        double Tl = 0.02;
        double alpha = 0.2;

        Complex j = new Complex(0.0, 1.0);

        List<Double> w = new ArrayList<>();
        int N = 600;
        for (int k = 0; k <= N; ++k) {
            double exponent = -1.0 + 4.0 * (double) k / (double) N;
            w.add(Math.pow(10.0, exponent));
        }

        double MS = 0.0;
        double robustIndex = 0.0;

        for (double wk : w) {
            Complex s = j.multiply(wk);

            Complex P = new Complex(K, 0.0).divide(
                    s.multiply(s).multiply(J).add(s.multiply(B))
            );

            Complex pi = new Complex(1.0, 0.0).add(
                    new Complex(1.0, 0.0).divide(s.multiply(Ti))
            );

            Complex lead = s.multiply(Tl).add(1.0)
                    .divide(s.multiply(alpha * Tl).add(1.0));

            Complex C = pi.multiply(lead).multiply(kp);

            Complex L = C.multiply(P);
            Complex S = new Complex(1.0, 0.0).divide(
                    new Complex(1.0, 0.0).add(L)
            );
            Complex T = L.divide(
                    new Complex(1.0, 0.0).add(L)
            );

            Complex W2 = (s.divide(50.0).add(1.0))
                    .divide(s.divide(500.0).add(1.0))
                    .multiply(0.2);

            double magS = S.abs();
            double magWT = W2.multiply(T).abs();

            if (magS > MS) {
                MS = magS;
            }
            if (magWT > robustIndex) {
                robustIndex = magWT;
            }
        }

        System.out.printf("Maximum sensitivity M_S ≈ %.2f%n", MS);
        System.out.printf("max |W2(jw) T(jw)| ≈ %.2f%n", robustIndex);
        if (robustIndex < 1.0) {
            System.out.println("Small-gain robust stability condition satisfied.");
        } else {
            System.out.println("Robust stability not guaranteed.");
        }
    }
}
