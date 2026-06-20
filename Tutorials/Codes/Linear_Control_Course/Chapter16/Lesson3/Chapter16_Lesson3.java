import org.apache.commons.math3.complex.Complex;

public class NicholsExample {

    static Complex Gofjw(double w) {
        Complex jw = new Complex(0.0, w);
        return Complex.ONE.divide(jw.multiply(jw.add(Complex.ONE)));
    }

    static Complex Kofjw(double w, double Kc, double tauZ, double tauP) {
        Complex jw = new Complex(0.0, w);
        Complex num = jw.multiply(tauZ).add(Complex.ONE);
        Complex den = jw.multiply(tauP).add(Complex.ONE);
        return num.divide(den).multiply(Kc);
    }

    public static void main(String[] args) {
        double Kc = 4.0;
        double tauZ = 0.5;
        double tauP = 0.05;

        for (int k = 0; k <= 400; ++k) {
            double w = Math.pow(10.0, -2.0 + 4.0 * k / 400.0);
            Complex L = Kofjw(w, Kc, tauZ, tauP).multiply(Gofjw(w));

            double mag = L.abs();
            double phase = Math.toDegrees(Math.atan2(L.getImaginary(), L.getReal()));
            double magdB = 20.0 * Math.log10(mag);

            System.out.printf("%e %f %f%n", w, magdB, phase);
        }
    }
}
