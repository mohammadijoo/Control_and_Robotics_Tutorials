import org.apache.commons.math3.complex.Complex;
import java.util.ArrayList;
import java.util.List;

public class STAnalysis {
    private static final double Kp = 4.0;
    private static final double Ki = 3.0;
    private static final Complex j = new Complex(0.0, 1.0);

    static Complex G(Complex s) {
        // G(s) = 1 / (s (s + 1))
        return Complex.ONE.divide(s.multiply(s.add(Complex.ONE)));
    }

    static Complex C(Complex s) {
        // C(s) = Kp + Ki / s
        return new Complex(Kp, 0.0).add(new Complex(Ki, 0.0).divide(s));
    }

    public static void main(String[] args) {
        List<Double> w = new ArrayList<>();
        for (int k = 0; k <= 400; ++k) {
            double wk = Math.pow(10.0, -2.0 + 4.0 * k / 400.0);
            w.add(wk);
        }

        double Ms = 0.0;
        double Mt = 0.0;
        for (double wk : w) {
            Complex s = j.multiply(wk);
            Complex L = C(s).multiply(G(s));
            Complex S = Complex.ONE.divide(Complex.ONE.add(L));
            Complex T = L.divide(Complex.ONE.add(L));
            double magS = S.abs();
            double magT = T.abs();
            Ms = Math.max(Ms, magS);
            Mt = Math.max(Mt, magT);
        }
        System.out.println("M_S = " + Ms + ", M_T = " + Mt);

        // This pattern can be integrated into higher-level Java robotics frameworks
        // for automatic verification of closed-loop designs.
    }
}
