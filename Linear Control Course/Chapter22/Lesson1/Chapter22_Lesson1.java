import org.apache.commons.math3.complex.Complex;

public class SensitivityPiExample {
    public static void main(String[] args) {
        double Kp = 5.0;
        double Ki = 10.0;
        double J  = 0.01;
        double B  = 0.1;

        Complex j = new Complex(0.0, 1.0);

        for (int k = 0; k <= 40; ++k) {
            double w = Math.pow(10.0, -1.0 + 0.1 * k); // 10^(-1) ... 10^3
            Complex s = j.multiply(w);

            // Plant P(s) = 1 / (J s + B)
            Complex P = new Complex(1.0, 0.0).divide(
                s.multiply(J).add(B)
            );

            // PI controller C(s) = Kp + Ki/s
            Complex C = new Complex(Kp, 0.0).add(
                new Complex(Ki, 0.0).divide(s)
            );

            Complex L = C.multiply(P);
            Complex S = new Complex(1.0, 0.0).divide(
                new Complex(1.0, 0.0).add(L)
            );

            double magS = S.abs();
            double magSdB = 20.0 * Math.log10(magS);

            System.out.printf("w = %.4f, |S(jw)| = %.4f (%.2f dB)%n",
                              w, magS, magSdB);
        }
    }
}
