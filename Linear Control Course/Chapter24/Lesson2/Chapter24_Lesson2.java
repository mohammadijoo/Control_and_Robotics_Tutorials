public final class Complex {
    public final double re, im;
    public Complex(double re, double im) { this.re = re; this.im = im; }
    public Complex add(Complex o) { return new Complex(re + o.re, im + o.im); }
    public Complex sub(Complex o) { return new Complex(re - o.re, im - o.im); }
    public Complex mul(Complex o) {
        return new Complex(re * o.re - im * o.im, re * o.im + im * o.re);
    }
    public Complex div(Complex o) {
        double den = o.re * o.re + o.im * o.im;
        return new Complex((re * o.re + im * o.im) / den,
                           (im * o.re - re * o.im) / den);
    }
    public double abs() { return Math.hypot(re, im); }
    public static final Complex ONE = new Complex(1.0, 0.0);
}

// Example: evaluate S(jw), T(jw) on a grid
public class RobustPerfCheck {

    static Complex Pjw(double w, double zeta, double wn) {
        Complex jw = new Complex(0.0, w);
        Complex denom = jw.mul(jw)
            .add(new Complex(2.0 * zeta * wn, 0.0).mul(jw))
            .add(new Complex(wn * wn, 0.0));
        return Complex.ONE.div(denom);
    }

    static Complex Cjw(double w, double K, double z) {
        Complex jw = new Complex(0.0, w);
        return new Complex(K, 0.0)
            .mul(jw.add(new Complex(z, 0.0)))
            .div(jw);
    }

    public static void main(String[] args) {
        double zeta = 0.7, wn = 10.0;
        double K = 20.0, z = 2.0;

        double maxS = 0.0, maxT = 0.0;
        for (int i = 0; i <= 500; ++i) {
            double frac = i / 500.0;
            double w = Math.pow(10.0, -2.0 + 5.0 * frac); // 1e-2..1e3

            Complex P = Pjw(w, zeta, wn);
            Complex C = Cjw(w, K, z);
            Complex L = C.mul(P);
            Complex denom = Complex.ONE.add(L);
            Complex S = Complex.ONE.div(denom);
            Complex T = L.div(denom);

            maxS = Math.max(maxS, S.abs());
            maxT = Math.max(maxT, T.abs());
        }

        System.out.println("max |S(jw)| = " + maxS);
        System.out.println("max |T(jw)| = " + maxT);
        // These can be compared with target bounds implied by weights W_P, W_N, W_D.
    }
}
