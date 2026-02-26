import java.util.List;

public class FRFEstimator {

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
        public Complex multiply(Complex other) {
            return new Complex(
                this.re * other.re - this.im * other.im,
                this.re * other.im + this.im * other.re
            );
        }
        public Complex divide(Complex other) {
            double denom = other.re * other.re + other.im * other.im;
            return new Complex(
                (this.re * other.re + this.im * other.im) / denom,
                (this.im * other.re - this.re * other.im) / denom
            );
        }
        public double abs() { return Math.hypot(re, im); }
        public double arg() { return Math.atan2(im, re); }
    }

    public static Complex estimateDFT(List<Double> x, double Ts, double omega) {
        int N = x.size();
        double re = 0.0;
        double im = 0.0;
        for (int k = 0; k < N; ++k) {
            double tk = k * Ts;
            double angle = -omega * tk;
            double c = Math.cos(angle);
            double s = Math.sin(angle);
            re += x.get(k) * c;
            im += x.get(k) * s;
        }
        return new Complex(re, im);
    }

    public static Complex estimateFRF(List<Double> u,
                                      List<Double> y,
                                      double Ts,
                                      double omega)
    {
        Complex U = estimateDFT(u, Ts, omega);
        Complex Y = estimateDFT(y, Ts, omega);
        return Y.divide(U);
    }

    public static void main(String[] args) {
        // Example usage; in practice, fill from robot logs.
        // ...
    }
}
