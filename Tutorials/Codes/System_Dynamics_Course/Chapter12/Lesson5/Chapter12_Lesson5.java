/*
Chapter 12 - Lesson 5: Time–Frequency Domain Relationships and Trade-offs
File: Chapter12_Lesson5.java

Java program to compute frequency metrics (Mr, wb) and underdamped step metrics (Mp, ts)
for the canonical 2nd-order low-pass:
    G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

This file is self-contained (no external libraries). For production work,
consider Apache Commons Math (complex numbers, solvers) or EJML (matrix ops).

Compile:
  javac Chapter12_Lesson5.java
Run:
  java Chapter12_Lesson5
*/

import java.util.*;

public class Chapter12_Lesson5 {

    static class Complex {
        final double re, im;
        Complex(double re, double im){ this.re = re; this.im = im; }
        Complex add(Complex b){ return new Complex(this.re + b.re, this.im + b.im); }
        Complex mul(Complex b){
            return new Complex(this.re*b.re - this.im*b.im, this.re*b.im + this.im*b.re);
        }
        Complex div(Complex b){
            double d = b.re*b.re + b.im*b.im;
            return new Complex((this.re*b.re + this.im*b.im)/d, (this.im*b.re - this.re*b.im)/d);
        }
        double abs(){ return Math.hypot(re, im); }
    }

    static Complex G_jw(double w, double zeta, double wn){
        // G(jw) = wn^2 / ((jw)^2 + 2 zeta wn (jw) + wn^2)
        Complex jw = new Complex(0.0, w);
        Complex jw2 = jw.mul(jw);
        Complex term = new Complex(0.0, 2.0*zeta*wn*w); // 2 zeta wn (jw)
        Complex den = jw2.add(term).add(new Complex(wn*wn, 0.0));
        return new Complex(wn*wn, 0.0).div(den);
    }

    static class FreqMetrics {
        double Mr, wr, wb;
        FreqMetrics(double Mr, double wr, double wb){ this.Mr = Mr; this.wr = wr; this.wb = wb; }
    }

    static FreqMetrics frequencyMetrics(double zeta, double wn){
        int N = 6000;
        double wmin = 1e-2 * wn;
        double wmax = 1e+3 * wn;

        double Mr = 0.0;
        double wr = wmin;

        double dc = G_jw(wmin, zeta, wn).abs();
        double target = dc / Math.sqrt(2.0);

        double wb = Double.NaN;
        boolean found = false;

        for(int i=0;i<N;i++){
            double a = (double)i/(N-1);
            double w = wmin * Math.pow(wmax/wmin, a);
            double mag = G_jw(w, zeta, wn).abs();

            if(mag > Mr){
                Mr = mag;
                wr = w;
            }
            if(!found && mag <= target){
                wb = w;
                found = true;
            }
        }
        return new FreqMetrics(Mr, wr, wb);
    }

    static double overshootPercent(double zeta){
        // Mp = exp(-zeta*pi/sqrt(1-zeta^2))*100%, valid for 0<zeta<1
        if(zeta <= 0.0 || zeta >= 1.0) return 0.0;
        return Math.exp(-zeta*Math.PI/Math.sqrt(1.0 - zeta*zeta))*100.0;
    }

    static double settlingTime2pct(double zeta, double wn){
        // ts(2%) ~ 4/(zeta*wn)
        return 4.0/(zeta*wn);
    }

    public static void main(String[] args){
        double zeta = 0.35;
        double wn   = 12.0;

        FreqMetrics fm = frequencyMetrics(zeta, wn);

        System.out.println("=== Java demo: 2nd-order time-frequency tradeoffs ===");
        System.out.printf(Locale.US, "zeta=%.3f, wn=%.3f rad/s%n", zeta, wn);
        System.out.printf(Locale.US, "Resonant peak Mr = %.6f%n", fm.Mr);
        System.out.printf(Locale.US, "Resonant freq  wr = %.6f rad/s%n", fm.wr);
        System.out.printf(Locale.US, "Bandwidth (-3dB) wb = %.6f rad/s%n", fm.wb);

        if(zeta > 0.0 && zeta < 1.0){
            System.out.printf(Locale.US, "Overshoot Mp = %.2f %% %n", overshootPercent(zeta));
            System.out.printf(Locale.US, "Settling time ts(2%%) ~ %.6f s%n", settlingTime2pct(zeta, wn));
        }

        System.out.println();
        System.out.println("=== Sweep zeta (fixed wn) ===");
        System.out.println("zeta, Mp(%), Mr, wb(rad/s)");
        double[] zetas = {0.15,0.25,0.35,0.50,0.70};
        for(double z: zetas){
            FreqMetrics fmi = frequencyMetrics(z, wn);
            double Mp = overshootPercent(z);
            System.out.printf(Locale.US, "%.2f, %.2f, %.4f, %.4f%n", z, Mp, fmi.Mr, fmi.wb);
        }
    }
}
