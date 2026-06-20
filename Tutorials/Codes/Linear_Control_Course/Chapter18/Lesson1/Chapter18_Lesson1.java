public class TrackingErrorExample {
    public static void main(String[] args) {
        double K = 50.0;
        double w0 = 1.0; // rad/s

        // s = j w0
        double sr = 0.0;
        double si = w0;

        // P(s) = 1 / (s (s + 1))
        Complex s = new Complex(sr, si);
        Complex P = Complex.one().divide(s.multiply(s.add(Complex.one())));
        Complex C = new Complex(K, 0.0);
        Complex L = C.multiply(P);

        Complex Gref = L.divide(Complex.one().add(L));   // Y/R
        Complex Gerr = Complex.one().divide(Complex.one().add(L)); // E/R

        System.out.println("At w = " + w0 + " rad/s");
        System.out.println("  |Y/R|  = " + Gref.abs());
        System.out.println("  |E/R|  = " + Gerr.abs());
    }

    // Minimal complex helper (in practice use Apache Commons Math or similar)
    static class Complex {
        final double re, im;
        Complex(double r, double i) { re = r; im = i; }

        static Complex one() { return new Complex(1.0, 0.0); }

        Complex add(Complex z) { return new Complex(re + z.re, im + z.im); }
        Complex multiply(Complex z) {
            return new Complex(re * z.re - im * z.im, re * z.im + im * z.re);
        }
        Complex divide(Complex z) {
            double d = z.re * z.re + z.im * z.im;
            return new Complex((re * z.re + im * z.im) / d,
                               (im * z.re - re * z.im) / d);
        }
        double abs() { return Math.hypot(re, im); }
    }
}
