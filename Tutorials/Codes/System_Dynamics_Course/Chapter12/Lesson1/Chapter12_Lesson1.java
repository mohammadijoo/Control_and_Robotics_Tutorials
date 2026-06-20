/*
Chapter12_Lesson1.java
System Dynamics (Control Engineering) — Chapter 12, Lesson 1
Sinusoidal Steady-State Response and Frequency Response Definition

This program:
1) Evaluates a 2nd-order transfer function at s = j*omega to obtain G(jw).
2) Simulates the equivalent ODE with RK4 under sinusoidal forcing.
3) Fits the steady-state output to extract amplitude and phase.

Compile and run:
  javac Chapter12_Lesson1.java
  java Chapter12_Lesson1

No external dependencies.
*/

import java.util.ArrayList;
import java.util.List;

public class Chapter12_Lesson1 {

    static final double PI = 3.14159265358979323846;

    // Minimal complex class
    static class Complex {
        final double re, im;

        Complex(double re, double im) { this.re = re; this.im = im; }

        Complex add(Complex z) { return new Complex(this.re + z.re, this.im + z.im); }
        Complex sub(Complex z) { return new Complex(this.re - z.re, this.im - z.im); }
        Complex mul(Complex z) {
            return new Complex(this.re*z.re - this.im*z.im, this.re*z.im + this.im*z.re);
        }
        Complex div(Complex z) {
            double d = z.re*z.re + z.im*z.im;
            return new Complex((this.re*z.re + this.im*z.im)/d, (this.im*z.re - this.re*z.im)/d);
        }
        double abs() { return Math.hypot(this.re, this.im); }
        double arg() { return Math.atan2(this.im, this.re); }

        public String toString() {
            return String.format("(%.6f %+.6f j)", re, im);
        }
    }

    static Complex polyval(double[] c, Complex s) {
        // c in descending powers
        Complex y = new Complex(0.0, 0.0);
        for (double a : c) {
            y = y.mul(s).add(new Complex(a, 0.0));
        }
        return y;
    }

    static Complex tfEval(double[] num, double[] den, Complex s) {
        return polyval(num, s).div(polyval(den, s));
    }

    static class FitResult {
        final double R, phi, a, b;
        FitResult(double R, double phi, double a, double b) {
            this.R = R; this.phi = phi; this.a = a; this.b = b;
        }
    }

    // Least squares fit y ≈ a sin(wt) + b cos(wt)
    static FitResult fitSinusoid(List<Double> t, List<Double> y, double omega) {
        double s11 = 0.0, s12 = 0.0, s22 = 0.0;
        double r1  = 0.0, r2  = 0.0;

        for (int i = 0; i < t.size(); i++) {
            double si = Math.sin(omega * t.get(i));
            double ci = Math.cos(omega * t.get(i));
            double yi = y.get(i);

            s11 += si * si;
            s12 += si * ci;
            s22 += ci * ci;
            r1  += si * yi;
            r2  += ci * yi;
        }

        double det = s11 * s22 - s12 * s12;
        double a = ( r1 * s22 - r2 * s12) / det;
        double b = (-r1 * s12 + r2 * s11) / det;

        double R = Math.sqrt(a*a + b*b);
        double phi = Math.atan2(b, a);

        return new FitResult(R, phi, a, b);
    }

    static double wrapToPi(double x) {
        while (x >  PI) x -= 2.0*PI;
        while (x < -PI) x += 2.0*PI;
        return x;
    }

    public static void main(String[] args) {
        // Second-order low-pass: G(s)=wn^2/(s^2+2*zeta*wn*s+wn^2)
        double wn   = 5.0;
        double zeta = 0.2;

        double[] num = new double[]{wn*wn};
        double[] den = new double[]{1.0, 2.0*zeta*wn, wn*wn};

        // Input: u(t) = Um sin(omega t + phi_u)
        double Um = 1.0;
        double omega = 4.0;
        double phi_u = 0.0;

        Complex Gjw = tfEval(num, den, new Complex(0.0, omega));
        double Ym_pred = Gjw.abs() * Um;
        double phi_y_pred = Gjw.arg() + phi_u;

        System.out.println("G(jw) = " + Gjw);
        System.out.println("|G(jw)| = " + Gjw.abs() + "  angle(G(jw)) = " + Gjw.arg());
        System.out.println("Predicted steady-state amplitude Ym = " + Ym_pred);
        System.out.println("Predicted steady-state phase phi_y (rad) = " + phi_y_pred);
        System.out.println();

        // ODE: y'' + 2*zeta*wn*y' + wn^2*y = wn^2*u(t)
        // State: x1=y, x2=y'
        double t0 = 0.0, tf = 40.0, dt = 0.001;
        int N = (int)Math.round((tf - t0)/dt) + 1;

        List<Double> tvec = new ArrayList<>(N);
        List<Double> yvec = new ArrayList<>(N);

        double x1 = 0.0, x2 = 0.0;
        double t = t0;

        for (int k = 0; k < N; k++) {
            tvec.add(t);
            yvec.add(x1);

            // RK4
            double[] k1 = f(t, x1, x2, Um, omega, phi_u, wn, zeta);
            double[] k2 = f(t + 0.5*dt, x1 + 0.5*dt*k1[0], x2 + 0.5*dt*k1[1], Um, omega, phi_u, wn, zeta);
            double[] k3 = f(t + 0.5*dt, x1 + 0.5*dt*k2[0], x2 + 0.5*dt*k2[1], Um, omega, phi_u, wn, zeta);
            double[] k4 = f(t + dt, x1 + dt*k3[0], x2 + dt*k3[1], Um, omega, phi_u, wn, zeta);

            x1 += (dt/6.0) * (k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]);
            x2 += (dt/6.0) * (k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]);

            t += dt;
        }

        // Fit last 10 seconds
        List<Double> tss = new ArrayList<>();
        List<Double> yss = new ArrayList<>();
        for (int i = 0; i < tvec.size(); i++) {
            if (tvec.get(i) >= tf - 10.0) {
                tss.add(tvec.get(i));
                yss.add(yvec.get(i));
            }
        }

        FitResult fr = fitSinusoid(tss, yss, omega);

        System.out.println("Estimated from simulation (last 10 s):");
        System.out.println("Ym_hat = " + fr.R);
        System.out.println("phi_y_hat (rad) = " + wrapToPi(fr.phi));
        System.out.println();

        System.out.println("Errors:");
        System.out.println("Amplitude error: " + (fr.R - Ym_pred));
        System.out.println("Phase error (rad): " + wrapToPi(fr.phi - phi_y_pred));
    }

    static double[] f(double t, double x1, double x2,
                      double Um, double omega, double phi_u,
                      double wn, double zeta) {
        double u = Um * Math.sin(omega*t + phi_u);
        double dx1 = x2;
        double dx2 = -2.0*zeta*wn*x2 - wn*wn*x1 + wn*wn*u;
        return new double[]{dx1, dx2};
    }
}
