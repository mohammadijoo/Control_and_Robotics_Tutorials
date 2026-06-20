/* Chapter12_Lesson4.java
   Nyquist and Nichols Plots (Introductory Level) — Frequency Response and Resonance

   Dependencies (Maven):
     <dependency>
       <groupId>org.apache.commons</groupId>
       <artifactId>commons-math3</artifactId>
       <version>3.6.1</version>
     </dependency>

   Compile/run (example):
     javac -cp commons-math3-3.6.1.jar Chapter12_Lesson4.java
     java  -cp .:commons-math3-3.6.1.jar Chapter12_Lesson4

   Output:
     - Chapter12_Lesson4_freqresp_java.csv
*/

import org.apache.commons.math3.complex.Complex;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter12_Lesson4 {

    static Complex polyEval(double[] coeffs, Complex s) {
        // Horner evaluation: coeffs highest power first
        Complex y = Complex.ZERO;
        for (double c : coeffs) {
            y = y.multiply(s).add(new Complex(c, 0.0));
        }
        return y;
    }

    static double[] logspace(double a, double b, int n) {
        // 10^a to 10^b
        double[] w = new double[n];
        for (int k = 0; k < n; k++) {
            double t = (n == 1) ? 0.0 : (double) k / (double) (n - 1);
            double p = a + (b - a) * t;
            w[k] = Math.pow(10.0, p);
        }
        return w;
    }

    public static void main(String[] args) throws IOException {
        // Example open-loop plant:
        // G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * 1/(tau*s + 1)
        double K = 5.0;
        double wn = 10.0;
        double zeta = 0.20;
        double tau = 0.05;

        // num(s) = K*wn^2
        double[] num = new double[]{K * wn * wn};

        // den(s) = (s^2 + a1*s + a0)(b1*s + b0) expanded
        double a1 = 2.0 * zeta * wn;
        double a0 = wn * wn;
        double b1 = tau;
        double b0 = 1.0;

        double[] den = new double[]{
                b1,
                (b0 + a1 * b1),
                (a1 * b0 + a0 * b1),
                (a0 * b0)
        };

        int N = 1500;
        double[] w = logspace(-1.0, 2.5, N);

        try (PrintWriter pw = new PrintWriter(new FileWriter("Chapter12_Lesson4_freqresp_java.csv"))) {
            pw.println("omega_rad_s,ReG,ImG,Mag_dB,Phase_deg");
            for (double wk : w) {
                Complex s = new Complex(0.0, wk); // j*w
                Complex Gjw = polyEval(num, s).divide(polyEval(den, s));

                double mag = Gjw.abs();
                double magDb = 20.0 * Math.log10(mag);
                double phaseDeg = Math.atan2(Gjw.getImaginary(), Gjw.getReal()) * 180.0 / Math.PI;

                pw.printf(java.util.Locale.US, "%.10f,%.10f,%.10f,%.10f,%.10f%n",
                        wk, Gjw.getReal(), Gjw.getImaginary(), magDb, phaseDeg);
            }
        }

        System.out.println("Wrote Chapter12_Lesson4_freqresp_java.csv");
        System.out.println("Plot Nyquist: ReG vs ImG. Plot Nichols: Phase_deg vs Mag_dB.");
    }
}
