class Complex {
    public double re;
    public double im;

    public Complex(double re, double im) {
        this.re = re;
        this.im = im;
    }
    public Complex add(Complex other) {
        return new Complex(this.re + other.re, this.im + other.im);
    }
    public Complex sub(Complex other) {
        return new Complex(this.re - other.re, this.im - other.im);
    }
    public Complex mul(Complex other) {
        return new Complex(
            this.re * other.re - this.im * other.im,
            this.re * other.im + this.im * other.re
        );
    }
    public Complex inv() {
        double den = re * re + im * im;
        return new Complex(re / den, -im / den);
    }
    public double abs() {
        return Math.hypot(re, im);
    }
}

public class LoopGainST {
    public static void main(String[] args) {
        double J = 0.01;
        double B = 0.1;
        double Kp = 50.0;
        double Kd = 2.0;

        double w = 10.0; // example frequency [rad/s]
        Complex jw = new Complex(0.0, w);

        // P(jw) = 1 / (J (jw)^2 + B jw)
        Complex denomP = new Complex(
                -J * w * w,  // real part of J (jw)^2 + B jw
                B * w       // imag part
        );
        Complex P = denomP.inv();

        // C(jw) = Kp + Kd jw
        Complex C = new Complex(Kp, Kd * w);

        Complex L = C.mul(P);
        Complex one = new Complex(1.0, 0.0);

        Complex S = one.add(L).inv();
        Complex T = L.mul(S);

        System.out.println("|S(jw)| = " + S.abs());
        System.out.println("|T(jw)| = " + T.abs());
    }
}
