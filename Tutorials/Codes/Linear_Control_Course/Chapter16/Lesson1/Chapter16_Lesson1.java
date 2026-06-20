public class NicholsBasic {

    // Minimal complex class for frequency response computations
    static class Complex {
        final double re;
        final double im;

        Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        Complex add(Complex other) {
            return new Complex(this.re + other.re, this.im + other.im);
        }

        Complex mul(Complex other) {
            return new Complex(
                this.re * other.re - this.im * other.im,
                this.re * other.im + this.im * other.re
            );
        }

        Complex div(Complex other) {
            double denom = other.re * other.re + other.im * other.im;
            return new Complex(
                (this.re * other.re + this.im * other.im) / denom,
                (this.im * other.re - this.re * other.im) / denom
            );
        }

        double abs() {
            return Math.hypot(re, im);
        }

        double arg() {
            return Math.atan2(im, re);
        }
    }

    public static void main(String[] args) {
        double K = 10.0;
        double T = 0.1;

        double wMin = 0.1;
        double wMax = 1000.0;
        int N = 200;

        for (int k = 0; k < N; ++k) {
            double alpha = (double) k / (double) (N - 1);
            double logW = Math.log10(wMin) + alpha * (Math.log10(wMax) - Math.log10(wMin));
            double w = Math.pow(10.0, logW);

            Complex s = new Complex(0.0, w); // s = j w
            Complex denom = s.mul(new Complex(1.0, 0.0).add(s.mul(new Complex(T, 0.0))));
            Complex L = new Complex(K, 0.0).div(denom);

            double mag = L.abs();
            double magDb = 20.0 * Math.log10(mag);
            double phaseDeg = 180.0 / Math.PI * L.arg();

            System.out.println(phaseDeg + " " + magDb);
        }
    }
}
