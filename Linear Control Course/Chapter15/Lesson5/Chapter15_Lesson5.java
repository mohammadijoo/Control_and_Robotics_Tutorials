import org.apache.commons.math3.complex.Complex;

public class NyquistExample {

    // G(s) = 1 / (s (s + 1) (s + 3))
    static Complex G_of_s(Complex s) {
        Complex term1 = s;
        Complex term2 = s.add(1.0);
        Complex term3 = s.add(3.0);
        return Complex.ONE.divide(term1.multiply(term2).multiply(term3));
    }

    public static void main(String[] args) {
        double K = 9.0; // proportional gain candidate

        double wMin = 1e-2;
        double wMax = 1e2;
        int N = 400;

        for (int k = 0; k < N; ++k) {
            double alpha = (double) k / (double) (N - 1);
            double w = wMin * Math.pow(wMax / wMin, alpha);
            Complex s = new Complex(0.0, w); // j * w
            Complex L = G_of_s(s).multiply(K);
            System.out.printf("%f %f%n", L.getReal(), L.getImaginary());
        }

        // These (Re, Im) pairs can be plotted in a tool or used within
        // robot code to verify that the Nyquist curve stays away from -1.
        // In WPILib-based code, K would be the proportional gain of a
        // joint PID controller.
    }
}
