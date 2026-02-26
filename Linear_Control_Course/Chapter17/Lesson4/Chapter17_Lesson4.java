public class RobotJointMargins {

    // Simple complex number class for frequency response
    static class Complex {
        final double re;
        final double im;
        Complex(double re, double im) { this.re = re; this.im = im; }

        Complex add(Complex other) {
            return new Complex(this.re + other.re, this.im + other.im);
        }

        Complex sub(Complex other) {
            return new Complex(this.re - other.re, this.im - other.im);
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

        double abs() { return Math.hypot(re, im); }
        double arg() { return Math.atan2(im, re); } // radians
    }

    // Robot joint parameters
    static final double J = 0.01;
    static final double B = 0.1;
    static final double K = 1.0;
    static double Kp = 20.0;

    // L(s) = (Kp*K) / (J s^2 + B s)
    static Complex L_of_jw(double w) {
        Complex s = new Complex(0.0, w);
        Complex num = new Complex(Kp * K, 0.0);
        Complex den = new Complex(J * ( -w * w ), B * w); // J*(j w)^2 + B*(j w)
        return num.div(den);
    }

    public static void main(String[] args) {
        int N = 2000;
        double wMin = 0.1;
        double wMax = 1000.0;

        double gm = Double.POSITIVE_INFINITY;
        double pm = Double.NEGATIVE_INFINITY;
        double wPc = 0.0, wGc = 0.0;

        boolean foundPc = false, foundGc = false;

        double wPrev = wMin;
        Complex Lprev = L_of_jw(wPrev);
        double magPrev = Lprev.abs();
        double phasePrev = Lprev.arg();

        for (int i = 1; i < N; ++i) {
            double alpha = (double) i / (N - 1);
            double w = wMin * Math.pow(wMax / wMin, alpha);

            Complex Lw = L_of_jw(w);
            double mag = Lw.abs();
            double phase = Lw.arg();

            if (!foundGc && (magPrev - 1.0) * (mag - 1.0) <= 0.0) {
                wGc = w;
                foundGc = true;
                double pmRad = Math.PI + phase;
                pm = pmRad * 180.0 / Math.PI;
            }

            if (!foundPc && (phasePrev + Math.PI) * (phase + Math.PI) <= 0.0) {
                wPc = w;
                foundPc = true;
                gm = 1.0 / mag;
            }

            wPrev = w;
            magPrev = mag;
            phasePrev = phase;
        }

        double gmDb = 20.0 * Math.log10(gm);

        System.out.println("Approximate GM (linear): " + gm);
        System.out.println("Approximate GM (dB):    " + gmDb);
        System.out.println("Approximate PM (deg):   " + pm);
        System.out.println("w_pc (rad/s):           " + wPc);
        System.out.println("w_gc (rad/s):           " + wGc);

        if (wGc > 0.0) {
            double pmRad = pm * Math.PI / 180.0;
            double tauD = pmRad / wGc;
            System.out.println("Approximate delay margin tau_d ~ " + tauD + " s");
        }

        // In an FRC robot, the same logic can be combined with WPILib's
        // identified plant models to reason about margins for drivetrain
        // or arm joints.
    }
}
