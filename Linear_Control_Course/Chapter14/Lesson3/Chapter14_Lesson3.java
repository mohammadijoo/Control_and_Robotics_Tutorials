import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.complex.Complex;

public class BodeExample {
    public static void main(String[] args) {
        double K = 10.0;
        double z = 10.0;
        double wn = 20.0;
        double zeta = 0.3;
        double Tdelay = 0.02;

        List<Double> omega = new ArrayList<>();
        for (int k = 0; k <= 100; ++k) {
            double exp10 = 0.0 + 3.0 * k / 100.0;
            omega.add(Math.pow(10.0, exp10));
        }

        Complex j = new Complex(0.0, 1.0);

        for (double w : omega) {
            Complex s = j.multiply(w);

            Complex G_zero = Complex.ONE.add(s.divide(z));

            Complex denom = Complex.ONE
                    .add(s.multiply(2.0 * zeta / wn))
                    .add(s.divide(wn).multiply(s.divide(wn)));
            Complex G_poles = Complex.ONE.divide(denom);

            Complex G_delay = (new Complex(0.0, -w * Tdelay)).exp();

            Complex G = G_zero.multiply(G_poles).multiply(G_delay).multiply(K);

            double mag = G.abs();
            double magDB = 20.0 * Math.log10(mag);
            double phaseRad = G.getArgument();
            double phaseDeg = phaseRad * 180.0 / Math.PI;

            System.out.println(w + " " + magDB + " " + phaseDeg);
        }
    }
}
