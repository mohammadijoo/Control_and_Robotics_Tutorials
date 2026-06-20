// Chapter19_Lesson4.java
// 2^3 factorial ablation effect estimation (Java)

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

public class Chapter19_Lesson4 {

    static class Run {
        int z1, z2, z3; // coded levels: -1 off, +1 on
        double y;
        Run(int z1, int z2, int z3, double y) { this.z1 = z1; this.z2 = z2; this.z3 = z3; this.y = y; }
    }

    static double simulateScore(int z1, int z2, int z3, double stress, int seed) {
        Random rng = new Random(seed);
        double noise = rng.nextGaussian();

        // Main effects + interactions + stress interaction
        double y = 80.0
                + 4.5 * z1 + 3.8 * z2 + 2.9 * z3
                + 1.5 * z1 * z2 + 1.0 * z1 * z3 + 0.6 * z2 * z3
                - 11.0 * stress - 1.8 * stress * z1 - 1.2 * stress * z3
                + 1.2 * noise;

        if (y < 0.0) y = 0.0;
        if (y > 100.0) y = 100.0;
        return y;
    }

    static List<Run> designRuns(double stress, int reps) {
        int[] levels = {-1, 1};
        List<Run> out = new ArrayList<>();
        int seed = 1000;
        for (int z1 : levels) {
            for (int z2 : levels) {
                for (int z3 : levels) {
                    for (int r = 0; r < reps; r++) {
                        out.add(new Run(z1, z2, z3, simulateScore(z1, z2, z3, stress, seed++)));
                    }
                }
            }
        }
        return out;
    }

    static double effect(List<Run> runs, String term) {
        double acc = 0.0;
        for (Run run : runs) {
            int x;
            switch (term) {
                case "z1": x = run.z1; break;
                case "z2": x = run.z2; break;
                case "z3": x = run.z3; break;
                case "z1z2": x = run.z1 * run.z2; break;
                case "z1z3": x = run.z1 * run.z3; break;
                case "z2z3": x = run.z2 * run.z3; break;
                default: throw new IllegalArgumentException("unknown term");
            }
            acc += run.y * x;
        }
        // For balanced coded 2^k design: effect = 2 * mean(y*x)
        return 2.0 * acc / runs.size();
    }

    public static void main(String[] args) {
        double[] stressGrid = {0.0, 0.4, 0.8};
        int reps = 16;

        for (double s : stressGrid) {
            List<Run> runs = designRuns(s, reps);
            double mean = 0.0;
            for (Run r : runs) mean += r.y;
            mean /= runs.size();

            System.out.println("=== stress = " + s + " ===");
            System.out.printf(Locale.US, "mean score: %.3f%n", mean);
            System.out.printf(Locale.US, "effect(z1=ScanMatch): %.3f%n", effect(runs, "z1"));
            System.out.printf(Locale.US, "effect(z2=IMUFusion): %.3f%n", effect(runs, "z2"));
            System.out.printf(Locale.US, "effect(z3=Predictor): %.3f%n", effect(runs, "z3"));
            System.out.printf(Locale.US, "effect(z1z2): %.3f%n", effect(runs, "z1z2"));
            System.out.printf(Locale.US, "effect(z1z3): %.3f%n", effect(runs, "z1z3"));
            System.out.printf(Locale.US, "effect(z2z3): %.3f%n", effect(runs, "z2z3"));
            System.out.println();
        }
    }
}
