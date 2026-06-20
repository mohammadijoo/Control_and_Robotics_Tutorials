/*
Chapter 12 - Lesson 2: Bode Plots: Magnitude, Phase, Asymptotes, and Construction Rules
System Dynamics (Control Engineering)

This Java program computes G(jw) for a rational transfer function with real coefficients
and writes a CSV file for plotting.

Compile:
  javac Chapter12_Lesson2.java

Run:
  java Chapter12_Lesson2

Output:
  bode_output_java.csv with columns: w, mag_db, phase_deg_unwrapped
*/

import java.io.PrintWriter;
import java.util.Locale;

public class Chapter12_Lesson2 {

    // Minimal complex number utility (no external dependencies).
    static final class Complex {
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
            return new Complex(this.re * other.re - this.im * other.im,
                               this.re * other.im + this.im * other.re);
        }

        Complex div(Complex other) {
            double denom = other.re * other.re + other.im * other.im;
            return new Complex((this.re * other.re + this.im * other.im) / denom,
                               (this.im * other.re - this.re * other.im) / denom);
        }

        double abs() {
            return Math.hypot(this.re, this.im);
        }

        double phaseDeg() {
            return Math.atan2(this.im, this.re) * 180.0 / Math.PI;
        }
    }

    static Complex polyval(double[] c, Complex s) {
        Complex y = new Complex(0.0, 0.0);
        for (double a : c) {
            y = y.mul(s).add(new Complex(a, 0.0));
        }
        return y;
    }

    static double magDb(Complex z) {
        return 20.0 * Math.log10(z.abs());
    }

    static void unwrapPhase(double[] phDeg) {
        for (int k = 1; k < phDeg.length; ++k) {
            double d = phDeg[k] - phDeg[k - 1];
            if (d > 180.0) {
                for (int j = k; j < phDeg.length; ++j) phDeg[j] -= 360.0;
            } else if (d < -180.0) {
                for (int j = k; j < phDeg.length; ++j) phDeg[j] += 360.0;
            }
        }
    }

    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.ROOT);

        // Example transfer function:
        //   G(s) = 10 * (1 + s/1) / ( s * (1 + s/10) )
        // num: 10*(s + 1) -> [10, 10]
        // den: s*(0.1 s + 1) -> 0.1 s^2 + s + 0 -> [0.1, 1, 0]
        double[] num = new double[]{10.0, 10.0};
        double[] den = new double[]{0.1, 1.0, 0.0};

        int N = 2000;
        double wMin = 1e-2;
        double wMax = 1e3;

        double[] w = new double[N];
        double[] mag = new double[N];
        double[] ph = new double[N];

        for (int k = 0; k < N; ++k) {
            double t = (double) k / (double) (N - 1);
            double wk = wMin * Math.pow(wMax / wMin, t); // logspace
            w[k] = wk;

            Complex s = new Complex(0.0, wk); // jw
            Complex Gjw = polyval(num, s).div(polyval(den, s));

            mag[k] = magDb(Gjw);
            ph[k] = Gjw.phaseDeg();
        }

        unwrapPhase(ph);

        try (PrintWriter out = new PrintWriter("bode_output_java.csv")) {
            out.println("w,mag_db,phase_deg");
            for (int k = 0; k < N; ++k) {
                out.printf(Locale.ROOT, "%.12g,%.12g,%.12g%n", w[k], mag[k], ph[k]);
            }
        }

        System.out.println("Wrote bode_output_java.csv (" + N + " points).");
        System.out.println("Plot using your preferred tool (e.g., MATLAB, Python, Excel).");
    }
}
