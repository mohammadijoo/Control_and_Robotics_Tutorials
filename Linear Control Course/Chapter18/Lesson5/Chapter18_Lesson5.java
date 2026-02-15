import org.apache.commons.math3.complex.Complex;
import java.util.ArrayList;
import java.util.List;

public class FrequencyTradeOffs {

    // P(s) = 1 / (s (s + 1) (s + 2))
    static Complex P(Complex s) {
        return Complex.ONE.divide(s.multiply(s.add(Complex.ONE))
                                 .multiply(s.add(new Complex(2.0, 0.0))));
    }

    // C(s) = (Kp s + Ki) / s
    static Complex C(Complex s, double Kp, double Ki) {
        return (new Complex(Kp, 0.0).multiply(s)
                .add(new Complex(Ki, 0.0))).divide(s);
    }

    public static void main(String[] args) {
        double Kp1 = 2.0, Ki1 = 1.0;
        double Kp2 = 6.0, Ki2 = 4.0;

        int N = 100;
        double wmin = 0.01, wmax = 100.0;
        List<Double> w = new ArrayList<>();

        for (int i = 0; i < N; ++i) {
            double logw = Math.log10(wmin)
                        + (Math.log10(wmax) - Math.log10(wmin)) * i / (N - 1);
            w.add(Math.pow(10.0, logw));
        }

        System.out.println("w, |L1(jw)|, |L2(jw)|");
        for (double wi : w) {
            Complex jw = new Complex(0.0, wi);
            Complex Pval = P(jw);

            Complex C1 = C(jw, Kp1, Ki1);
            Complex C2 = C(jw, Kp2, Ki2);

            Complex L1 = C1.multiply(Pval);
            Complex L2 = C2.multiply(Pval);

            System.out.println(wi + ", " + L1.abs() + ", " + L2.abs());
        }

        // Step responses can be obtained by discretizing the system and using
        // numerical integration or control libraries built on top of EJML.
    }
}
