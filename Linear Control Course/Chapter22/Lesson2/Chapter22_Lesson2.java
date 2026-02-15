// Minimal complex class (or use a library if available)
class Complex {
    public final double re, im;
    public Complex(double re, double im) { this.re = re; this.im = im; }
    public Complex add(Complex o) { return new Complex(re + o.re, im + o.im); }
    public Complex sub(Complex o) { return new Complex(re - o.re, im - o.im); }
    public Complex mul(Complex o) {
        return new Complex(re * o.re - im * o.im, re * o.im + im * o.re);
    }
    public Complex div(Complex o) {
        double d = o.re * o.re + o.im * o.im;
        return new Complex(
            (re * o.re + im * o.im) / d,
            (im * o.re - re * o.im) / d
        );
    }
    public double abs() { return Math.hypot(re, im); }
}

public class ComplementarySensitivity {
    public static void main(String[] args) {
        double Kp = 2.0;
        double tau_p = 0.1;
        double Kc = 5.0;

        for (double w = 1.0; w <= 1000.0; w *= 10.0) {
            Complex s = new Complex(0.0, w);
            // P(s) = Kp / (tau_p s + 1)
            Complex denomP = new Complex(1.0 + tau_p * s.re, tau_p * s.im);
            // denomP is (1 + tau_p * s)
            Complex P = new Complex(Kp, 0.0).div(denomP);
            Complex C = new Complex(Kc, 0.0);
            Complex L = C.mul(P);
            Complex one = new Complex(1.0, 0.0);
            Complex T = L.div(one.add(L));
            double magT = T.abs();
            System.out.println("w = " + w + ", |T(jw)| = " + magT);
        }
    }
}
