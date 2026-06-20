public class CrossoverFrequencies {

    public static class Complex {
        public final double re;
        public final double im;
        public Complex(double re, double im) {
            this.re = re; this.im = im;
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
            double denom = other.re * other.re + other.im * other.im;
            return new Complex(
                (this.re * other.re + this.im * other.im) / denom,
                (this.im * other.re - this.re * other.im) / denom
            );
        }
        public double abs() {
            return Math.hypot(re, im);
        }
        public double arg() {
            return Math.atan2(im, re);
        }
    }

    // Evaluate polynomial c[0] + c[1] s + ... at s
    public static Complex polyEval(double[] c, Complex s) {
        Complex val = new Complex(0.0, 0.0);
        Complex p = new Complex(1.0, 0.0);
        for (double coeff : c) {
            val = new Complex(val.re + coeff * p.re, val.im + coeff * p.im);
            p = p.mul(s);
        }
        return val;
    }

    public static void main(String[] args) {
        double K = 10.0;
        double T = 0.1;
        double[] num = {K};
        double[] den = {0.0, 1.0, T}; // T s^2 + s

        int N = 2000;
        double wMin = 0.1;
        double wMax = 1000.0;
        double[] w = new double[N];
        for (int i = 0; i < N; ++i) {
            double alpha = (double) i / (double) (N - 1);
            w[i] = wMin * Math.pow(wMax / wMin, alpha);
        }

        double wGc = -1.0;
        double wPc = -1.0;

        double prevG = 0.0;
        double prevH = 0.0;
        int prevGSign = 0;
        int prevHSign = 0;
        boolean first = true;

        for (int i = 0; i < N; ++i) {
            Complex s = new Complex(0.0, w[i]);
            Complex L = polyEval(num, s).div(polyEval(den, s));
            double mag = L.abs();
            double phase = L.arg();

            double g = mag - 1.0;

            double phaseWrapped = Math.atan2(Math.sin(phase), Math.cos(phase));
            double h = phaseWrapped + Math.PI;

            int sg = (g > 0) ? 1 : (g < 0 ? -1 : 0);
            int sh = (h > 0) ? 1 : (h < 0 ? -1 : 0);

            if (!first) {
                if (wGc < 0.0 && sg != 0 && sg != prevGSign && prevGSign != 0) {
                    double t = prevG / (prevG - g);
                    wGc = w[i - 1] + t * (w[i] - w[i - 1]);
                }
                if (wPc < 0.0 && sh != 0 && sh != prevHSign && prevHSign != 0) {
                    double t = prevH / (prevH - h);
                    wPc = w[i - 1] + t * (w[i] - w[i - 1]);
                }
            } else {
                first = false;
            }

            prevG = g;
            prevH = h;
            prevGSign = sg;
            prevHSign = sh;
        }

        System.out.println("Approx gain crossover w_gc = " + wGc + " rad/s");
        System.out.println("Approx phase crossover w_pc = " + wPc + " rad/s");
    }
}
