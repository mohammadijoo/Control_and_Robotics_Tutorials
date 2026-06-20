\
/* Chapter16_Lesson1.java
   Sampling, aliasing, and zero-order hold data generation.
   Compile: javac Chapter16_Lesson1.java
   Run:     java Chapter16_Lesson1
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter16_Lesson1 {

    public static double aliasFrequency(double f0, double fs) {
        long m = Math.round(f0 / fs);
        double fa = Math.abs(f0 - m * fs);
        if (fa > fs / 2.0) {
            fa = fs - fa;
        }
        return fa;
    }

    public static void main(String[] args) {
        final double fs = 80.0;
        final double Ts = 1.0 / fs;
        final double f1 = 12.0;
        final double f2 = 55.0;
        final double A1 = 1.0;
        final double A2 = 0.7;
        final double duration = 0.25;

        final double fdense = 5000.0;
        final double dt = 1.0 / fdense;
        final int Nd = (int) (duration * fdense);
        final int Ns = (int) (duration * fs);

        double[] td = new double[Nd];
        double[] xCont = new double[Nd];
        double[] xZoh = new double[Nd];
        double[] ts = new double[Ns];
        double[] xSamp = new double[Ns];

        for (int k = 0; k < Ns; k++) {
            ts[k] = k * Ts;
            xSamp[k] = A1 * Math.sin(2.0 * Math.PI * f1 * ts[k]) +
                       A2 * Math.sin(2.0 * Math.PI * f2 * ts[k]);
        }

        for (int i = 0; i < Nd; i++) {
            td[i] = i * dt;
            xCont[i] = A1 * Math.sin(2.0 * Math.PI * f1 * td[i]) +
                       A2 * Math.sin(2.0 * Math.PI * f2 * td[i]);

            int k = (int) Math.floor(td[i] / Ts);
            if (k < 0) k = 0;
            if (k >= Ns) k = Ns - 1;
            xZoh[i] = xSamp[k];
        }

        try (PrintWriter out = new PrintWriter(new FileWriter("Chapter16_Lesson1_java_output.csv"))) {
            out.println("t_dense,x_cont,x_zoh");
            for (int i = 0; i < Nd; i++) {
                out.printf("%.8f,%.8f,%.8f%n", td[i], xCont[i], xZoh[i]);
            }
            out.println();
            out.println("k,ts,x_sample");
            for (int k = 0; k < Ns; k++) {
                out.printf("%d,%.8f,%.8f%n", k, ts[k], xSamp[k]);
            }
        } catch (IOException e) {
            System.err.println("Failed to write CSV: " + e.getMessage());
            return;
        }

        System.out.printf("Sampling frequency fs = %.1f Hz%n", fs);
        System.out.printf("Nyquist frequency = %.1f Hz%n", fs / 2.0);
        System.out.printf("Input component %.1f Hz aliases to %.1f Hz%n", f2, aliasFrequency(f2, fs));
        System.out.println("CSV written to Chapter16_Lesson1_java_output.csv");
    }
}
