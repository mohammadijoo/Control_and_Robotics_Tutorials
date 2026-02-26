public class BodeExample {

    public static class Complex {
        public double re;
        public double im;

        public Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        public Complex add(Complex other) {
            return new Complex(this.re + other.re, this.im + other.im);
        }

        public Complex mul(Complex other) {
            return new Complex(
                this.re * other.re - this.im * other.im,
                this.re * other.im + this.im * other.re
            );
        }

        public Complex div(Complex other) {
            double den = other.re * other.re + other.im * other.im;
            return new Complex(
                (this.re * other.re + this.im * other.im) / den,
                (this.im * other.re - this.re * other.im) / den
            );
        }

        public double abs() {
            return Math.hypot(re, im);
        }

        public double arg() {
            return Math.atan2(im, re);
        }
    }

    public static Complex G(Complex s) {
        // G(s) = 100 (s + 10) / (s (s + 1) (s + 100))
        Complex K = new Complex(100.0, 0.0);
        Complex num = s.add(new Complex(10.0, 0.0));
        Complex den = s.mul(s.add(new Complex(1.0, 0.0)))
                      .mul(s.add(new Complex(100.0, 0.0)));
        return K.mul(num).div(den);
    }

    public static void main(String[] args) {
        for (int k = -2; k <= 3; ++k) {
            for (int j = 0; j < 10; ++j) {
                double exp10 = k + j / 10.0;
                double w = Math.pow(10.0, exp10);
                Complex s = new Complex(0.0, w);
                Complex Gjw = G(s);

                double mag = Gjw.abs();
                double magDb = 20.0 * Math.log10(mag);
                double phase = Gjw.arg() * 180.0 / Math.PI;

                System.out.printf("%.4f %.3f %.3f%n", w, magDb, phase);
            }
        }
    }
}
