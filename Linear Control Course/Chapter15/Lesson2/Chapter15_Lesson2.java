class Complex {
    public final double re;
    public final double im;

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

    public double arg() {
        return Math.atan2(im, re);
    }
}

public class NyquistDemo {

    static Complex L(Complex s, double K) {
        Complex num = new Complex(K, 0.0);
        Complex den = (new Complex(1.0, 0.0)).add(s.mul(new Complex(1.1, 0.0)))
                        .add(new Complex(0.5, 0.0)); // Example polynomial
        return num.div(den);
    }

    public static void main(String[] args) {
        int N = 4000;
        double wMin = 1e-2;
        double wMax = 1e2;
        double K = 5.0;

        Complex[] F = new Complex[N];
        for (int k = 0; k < N; ++k) {
            double alpha = (double) k / (N - 1);
            double w = Math.pow(10.0,
                    Math.log10(wMin) + alpha * (Math.log10(wMax) - Math.log10(wMin)));
            Complex s = new Complex(0.0, w);
            Complex Ljw = L(s, K);
            F[k] = new Complex(1.0, 0.0).add(Ljw); // F(s) = 1 + L(s)
        }

        double totalPhaseChange = 0.0;
        for (int k = 1; k < N; ++k) {
            double phiPrev = F[k-1].arg();
            double phiCurr = F[k].arg();
            double dphi = phiCurr - phiPrev;
            if (dphi > Math.PI) dphi -= 2.0 * Math.PI;
            if (dphi < -Math.PI) dphi += 2.0 * Math.PI;
            totalPhaseChange += dphi;
        }

        double Nccw = totalPhaseChange / (2.0 * Math.PI);
        double Nclockwise = -Nccw;
        System.out.println("Approximate clockwise encirclements of -1: N = " + Nclockwise);
    }
}
