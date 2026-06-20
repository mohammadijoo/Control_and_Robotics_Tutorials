import org.apache.commons.math3.complex.Complex;

public class FrequencyResponseExample {
    // G(s) = K / (1 + T s)
    private static double K = 2.0;
    private static double T = 0.1;

    public static Complex GofS(Complex s) {
        return new Complex(K, 0.0).divide(
                new Complex(1.0, 0.0).add(s.multiply(T)));
    }

    public static void main(String[] args) {
        double[] omega = {0.1, 1.0, 10.0, 100.0}; // rad/s
        Complex j = new Complex(0.0, 1.0);

        for (double w : omega) {
            Complex s = j.multiply(w);
            Complex G = GofS(s);

            double mag = G.abs();
            double phase = G.getArgument(); // radians

            System.out.printf("w = %7.3f rad/s, |G(jw)| = %7.3f, phase = %7.3f rad%n",
                              w, mag, phase);
        }
    }
}
