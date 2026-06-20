import java.util.Arrays;
import java.util.List;
import org.apache.commons.math3.complex.Complex;

public class RootLocusDemo {
    // Char. poly: s^2 + 2 s + K = 0 for G(s) = 1 / (s (s + 2))

    public static void main(String[] args) {
        List<Double> gains = Arrays.asList(0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0);

        for (double K : gains) {
            double a2 = 1.0;
            double a1 = 2.0;
            double a0 = K;

            double disc = a1 * a1 - 4.0 * a2 * a0;
            Complex sqrtDisc = new Complex(disc, 0.0).sqrt();

            Complex s1 = new Complex(-a1, 0.0).add(sqrtDisc).divide(2.0 * a2);
            Complex s2 = new Complex(-a1, 0.0).subtract(sqrtDisc).divide(2.0 * a2);

            System.out.println("K = " + K + "  poles: " + s1 + ", " + s2);
        }
    }
}
