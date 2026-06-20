public final class Complex {
    public final double re;
    public final double im;

    public Complex(double re, double im) {
        this.re = re;
        this.im = im;
    }

    public double abs() {
        return Math.hypot(re, im);
    }

    public double arg() {
        return Math.atan2(im, re);
    }

    public Complex times(Complex other) {
        return new Complex(
            re * other.re - im * other.im,
            re * other.im + im * other.re
        );
    }

    public Complex scale(double k) {
        return new Complex(k * re, k * im);
    }

    public static Complex expi(double theta) {
        return new Complex(Math.cos(theta), Math.sin(theta));
    }

    @Override
    public String toString() {
        return String.format("(%.4f %+ .4fj)", re, im);
    }
}

// Usage example in some robotics-oriented control code:
public class PhasorDemo {
    public static void main(String[] args) {
        double A = 1.0;
        double phiUdeg = 30.0;
        double phiU = Math.toRadians(phiUdeg);
        double omega = 10.0;
        double tau = 0.05;

        Complex U = Complex.expi(phiU).scale(A); // U* = A e^{j phi}

        // H(j omega) = 1 / (1 + j tau omega)
        Complex denom = new Complex(1.0, tau * omega);
        double denomAbsSq = denom.abs() * denom.abs();
        Complex H = new Complex(denom.re / denomAbsSq, -denom.im / denomAbsSq);

        Complex Y = H.times(U);

        System.out.println("H(j omega) = " + H);
        System.out.println("Y* = " + Y);
        System.out.println("Gain |H| = " + H.abs());
        System.out.println("Phase(H) [deg] = " + Math.toDegrees(H.arg()));
    }
}
