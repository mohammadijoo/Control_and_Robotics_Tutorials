/*
Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
Lesson 5 - Intro to Experimental Modal Analysis and System Identification Concepts

File: Chapter13_Lesson5.java

This Java example demonstrates a minimal FRF (H1) estimation from measured input u[n]
and output y[n] using a direct DFT (O(N^2)). It is intentionally simple and
educational.

For realistic work:
- Use FFT libraries (e.g., JTransforms) and Welch averaging.
- Use linear algebra libraries for MIMO and state-space identification.

Compile & run:
  javac Chapter13_Lesson5.java
  java Chapter13_Lesson5
*/

import java.util.Random;

public class Chapter13_Lesson5 {

    static final double PI = Math.PI;

    static class Complex {
        double re, im;
        Complex(double re, double im) { this.re = re; this.im = im; }
        Complex add(Complex b) { return new Complex(this.re + b.re, this.im + b.im); }
        Complex mul(Complex b) {
            return new Complex(this.re*b.re - this.im*b.im, this.re*b.im + this.im*b.re);
        }
        Complex conj() { return new Complex(this.re, -this.im); }
        double abs() { return Math.hypot(re, im); }
        double norm() { return re*re + im*im; }
        Complex div(Complex b) {
            double den = b.re*b.re + b.im*b.im + 1e-30;
            return new Complex((this.re*b.re + this.im*b.im)/den, (this.im*b.re - this.re*b.im)/den);
        }
        static Complex expj(double ang) { return new Complex(Math.cos(ang), Math.sin(ang)); }
    }

    static Complex[] dft(double[] x) {
        int N = x.length;
        Complex[] X = new Complex[N];
        for (int k = 0; k < N; k++) {
            Complex s = new Complex(0.0, 0.0);
            for (int n = 0; n < N; n++) {
                double ang = -2.0 * PI * k * n / (double)N;
                Complex w = Complex.expj(ang);
                s = s.add(new Complex(x[n], 0.0).mul(w));
            }
            X[k] = s;
        }
        return X;
    }

    public static void main(String[] args) {
        double fs = 500.0;
        double T = 8.0;
        int N = (int)Math.floor(fs * T);

        // Example "measured" signals (for demo only):
        // u: band-limited noise; y: u passed through a lightly damped 2nd-order filter + noise.
        double[] u = new double[N];
        double[] y = new double[N];
        Random rng = new Random(7);

        for (int i = 0; i < N; i++) u[i] = rng.nextGaussian();

        // simple smoothing lowpass
        int W = 9;
        double[] uLP = new double[N];
        for (int i = 0; i < N; i++) {
            double s = 0.0; int cnt = 0;
            for (int j = -W; j <= W; j++) {
                int k = i + j;
                if (0 <= k && k < N) { s += u[k]; cnt++; }
            }
            uLP[i] = s / (double)cnt;
        }
        u = uLP;

        // 2nd-order resonant filter (biquad-like) to mimic a mode
        double fn = 18.0;  // Hz
        double zeta = 0.03;
        double w0 = 2.0 * PI * fn;
        double dt = 1.0 / fs;

        double x1 = 0.0, x2 = 0.0; // states: position and velocity
        for (int i = 0; i < N; i++) {
            double f = u[i];

            // x1dot = x2
            // x2dot = -2 zeta w0 x2 - w0^2 x1 + f
            double k1_1 = x2;
            double k1_2 = -2.0*zeta*w0*x2 - w0*w0*x1 + f;

            double x1m = x1 + 0.5*dt*k1_1;
            double x2m = x2 + 0.5*dt*k1_2;
            double k2_1 = x2m;
            double k2_2 = -2.0*zeta*w0*x2m - w0*w0*x1m + f;

            x1 += dt*k2_1;
            x2 += dt*k2_2;

            double acc = -2.0*zeta*w0*x2 - w0*w0*x1 + f;
            y[i] = acc + 0.02*rng.nextGaussian();
        }

        // DFT-based spectra (single segment)
        Complex[] U = dft(u);
        Complex[] Y = dft(y);
        int K = N/2 + 1;

        System.out.println("f_Hz, |H1|, coherence");
        for (int k = 1; k < K && (fs*k/(double)N) <= 120.0; k++) {
            Complex Guu = U[k].mul(U[k].conj()).mul(new Complex(1.0/N, 0.0));
            Complex Gyy = Y[k].mul(Y[k].conj()).mul(new Complex(1.0/N, 0.0));
            Complex Gyu = Y[k].mul(U[k].conj()).mul(new Complex(1.0/N, 0.0));

            Complex H1 = Gyu.div(Guu);
            double coh = Gyu.norm() / ( (Guu.re*Gyy.re) + 1e-30 );

            double f = fs*k/(double)N;
            System.out.println(f + ", " + H1.abs() + ", " + coh);
        }
        System.out.println("Done.");
    }
}
