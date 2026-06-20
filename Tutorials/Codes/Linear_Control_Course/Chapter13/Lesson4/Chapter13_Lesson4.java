public class SecondOrderAnalysis {

    static class Complex {
        double re, im;
        Complex(double r, double i) { re = r; im = i; }

        Complex add(Complex o) { return new Complex(re + o.re, im + o.im); }
        Complex mul(Complex o) {
            return new Complex(re*o.re - im*o.im, re*o.im + im*o.re);
        }
        Complex mul(double k) { return new Complex(k*re, k*im); }
        Complex div(Complex o) {
            double den = o.re*o.re + o.im*o.im;
            return new Complex((re*o.re + im*o.im)/den,
                               (im*o.re - re*o.im)/den);
        }
        double abs() { return Math.hypot(re, im); }
    }

    public static void main(String[] args) {
        double zeta = 0.6;
        double wn   = 13.5;

        // Time simulation parameters
        double dt = 0.001;
        double T_end = 2.0;
        int N = (int)(T_end / dt);
        double[] t = new double[N];
        double[] y = new double[N];

        double yVal = 0.0, yDot = 0.0;
        double u = 1.0;

        for (int k = 0; k < N; ++k) {
            t[k] = k * dt;
            double yDDot = wn*wn * (u - yVal) - 2.0*zeta*wn*yDot;
            yDot += dt * yDDot;
            yVal += dt * yDot;
            y[k] = yVal;
        }

        // Overshoot and settling time (2% band)
        double yFinal = y[N-1];
        double peak = y[0];
        for (double v : y) {
            if (v > peak) peak = v;
        }
        double Mp = (peak - yFinal) / yFinal;

        int lastOutside = 0;
        for (int k = 0; k < N; ++k) {
            if (Math.abs(y[k] - yFinal) > 0.02 * Math.abs(yFinal)) {
                lastOutside = k;
            }
        }
        double ts = t[lastOutside];

        System.out.printf("Mp = %.2f%%%n", Mp * 100.0);
        System.out.printf("ts = %.3f s%n", ts);

        // Frequency response: scan omega
        double Mr = 0.0;
        double wr = 0.0;
        double target = 1.0 / Math.sqrt(2.0);
        Double wB = null;

        for (double w = 0.1; w <= 100.0; w *= 1.05) {
            Complex s = new Complex(0.0, w);
            Complex num = new Complex(wn*wn, 0.0);
            Complex den = s.mul(s).add(
                           new Complex(2.0*zeta*wn, 0.0).mul(s))
                         .add(new Complex(wn*wn, 0.0));
            Complex Tjw = num.div(den);
            double mag = Tjw.abs();

            if (mag > Mr) {
                Mr = mag;
                wr = w;
            }
            if (wB == null && mag <= target) {
                wB = w;
            }
        }
        System.out.printf("Mr = %.3f, wr = %.3f rad/s%n", Mr, wr);
        System.out.printf("wB (approx) = %.3f rad/s%n", wB);
    }
}
