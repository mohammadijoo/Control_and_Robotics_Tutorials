import org.apache.commons.math3.complex.Complex;

public class NyquistRHPDelay {

    public static Complex Lofjw(double w, double K, double p, double Ldelay) {
        Complex s = new Complex(0.0, w);          // s = j w
        Complex G0 = new Complex(K, 0.0).divide(s.subtract(new Complex(p, 0.0)));
        Complex delay = (new Complex(0.0, -w * Ldelay)).exp(); // exp(-j w L)
        return G0.multiply(delay);
    }

    public static void main(String[] args) {
        double K = 2.0;
        double p = 1.0;
        double Ldelay = 0.03; // 30 ms

        for (int k = 0; k <= 500; ++k) {
            double w = 0.1 * k;
            Complex L = Lofjw(w, K, p, Ldelay);
            System.out.printf("%f %f %f%n", w, L.getReal(), L.getImaginary());
        }
    }
}
