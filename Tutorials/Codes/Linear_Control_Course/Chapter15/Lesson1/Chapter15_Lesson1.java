import org.apache.commons.math3.complex.Complex;
import java.util.ArrayList;
import java.util.List;

public class NyquistExample {

    static double J = 0.01;
    static double B = 0.1;
    static double K = 1.0;
    static double Kp = 5.0;

    static Complex L_of_s(Complex s) {
        Complex numerator = new Complex(Kp * K, 0.0);
        Complex denominator = s.multiply(s).multiply(J).add(s.multiply(B));
        return numerator.divide(denominator);
    }

    public static void main(String[] args) {
        int N = 400;
        double wMin = 1e-2;
        double wMax = 1e3;

        List<Complex> nyquistPoints = new ArrayList<>(2 * N);

        // Positive frequencies
        for (int k = 0; k < N; ++k) {
            double alpha = (double) k / (double) (N - 1);
            double w = wMin * Math.pow(wMax / wMin, alpha);
            Complex s = new Complex(0.0, w);
            nyquistPoints.add(L_of_s(s));
        }

        // Conjugate branch
        for (int k = N - 1; k >= 0; --k) {
            nyquistPoints.add(nyquistPoints.get(k).conjugate());
        }

        // Export or plot results
        for (Complex z : nyquistPoints) {
            System.out.println(z.getReal() + "," + z.getImaginary());
        }
    }
}
