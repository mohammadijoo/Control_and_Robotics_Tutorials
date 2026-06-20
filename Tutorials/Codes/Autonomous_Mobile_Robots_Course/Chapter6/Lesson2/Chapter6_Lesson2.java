/*
Chapter6_Lesson2.java
Bayes Filters for Mobile Robots (discrete-state version)

Implements:
  bel_bar[i] = sum_j T[i][j] * bel_prev[j]
  bel[i]     = eta * L[i] * bel_bar[i]

Compile/Run:
  javac Chapter6_Lesson2.java
  java Chapter6_Lesson2
*/

import java.util.Arrays;
import java.util.Random;

public class Chapter6_Lesson2 {

    static double[] normalize(double[] p) {
        double s = 0.0;
        for (double v : p) s += v;
        if (s < 1e-12) {
            double[] u = new double[p.length];
            Arrays.fill(u, 1.0 / p.length);
            return u;
        }
        double[] out = new double[p.length];
        for (int i = 0; i < p.length; i++) out[i] = p[i] / s;
        return out;
    }

    static double[] predict(double[] belPrev, double[][] T) {
        int N = belPrev.length;
        double[] belBar = new double[N];
        for (int i = 0; i < N; i++) {
            double s = 0.0;
            for (int j = 0; j < N; j++) s += T[i][j] * belPrev[j];
            belBar[i] = s;
        }
        return normalize(belBar);
    }

    static double[] update(double[] belBar, double[] L) {
        int N = belBar.length;
        double[] bel = new double[N];
        for (int i = 0; i < N; i++) bel[i] = L[i] * belBar[i];
        return normalize(bel);
    }

    static double[][] makeShiftTransition(int N, int delta, double pExact, double pUnder, double pOver) {
        if (Math.abs((pExact + pUnder + pOver) - 1.0) > 1e-9) {
            throw new IllegalArgumentException("Probabilities must sum to 1");
        }
        double[][] T = new double[N][N];
        for (int j = 0; j < N; j++) {
            int iExact = mod(j + delta, N);
            int iUnder = mod(j + delta - 1, N);
            int iOver  = mod(j + delta + 1, N);
            T[iExact][j] += pExact;
            T[iUnder][j] += pUnder;
            T[iOver][j]  += pOver;
        }
        return T;
    }

    static double[] landmarkLikelihood(int N, int z, int[] landmarks, double pHit, double pFalse) {
        double[] L = new double[N];
        double base = (z == 1) ? pFalse : (1.0 - pFalse);
        Arrays.fill(L, base);
        for (int lm : landmarks) {
            int x = mod(lm, N);
            L[x] = (z == 1) ? pHit : (1.0 - pHit);
        }
        return L;
    }

    static int mod(int a, int n) {
        int r = a % n;
        return (r < 0) ? r + n : r;
    }

    public static void main(String[] args) {
        int N = 20;
        double[] bel = new double[N];
        Arrays.fill(bel, 1.0 / N);

        int[] landmarks = new int[]{3, 14};
        int[] controls  = new int[]{1, 1, 1, 2, 1, 1, 2, 1};

        Random rng = new Random(4);
        int x = rng.nextInt(N);
        int[] truth = new int[controls.length + 1];
        truth[0] = x;
        int[] zs = new int[controls.length];

        // Simulate truth and measurements
        for (int t = 0; t < controls.length; t++) {
            int u = controls[t];
            double r = rng.nextDouble();
            if (r < 0.8) x = mod(x + u, N);
            else if (r < 0.9) x = mod(x + u - 1, N);
            else x = mod(x + u + 1, N);
            truth[t + 1] = x;

            boolean atLandmark = (x == landmarks[0] || x == landmarks[1]);
            int z;
            if (atLandmark) z = (rng.nextDouble() < 0.9) ? 1 : 0;
            else z = (rng.nextDouble() < 0.1) ? 1 : 0;
            zs[t] = z;
        }

        // Filter loop
        for (int t = 0; t < controls.length; t++) {
            int u = controls[t];
            int z = zs[t];
            double[][] T = makeShiftTransition(N, u, 0.8, 0.1, 0.1);
            double[] L = landmarkLikelihood(N, z, landmarks, 0.9, 0.1);
            double[] belBar = predict(bel, T);
            bel = update(belBar, L);
        }

        // Print top-5
        Integer[] idx = new Integer[N];
        for (int i = 0; i < N; i++) idx[i] = i;
        Arrays.sort(idx, (a, b) -> Double.compare(bel[b], bel[a]));

        System.out.println("Final belief (top 5 states):");
        for (int k = 0; k < 5; k++) {
            int i = idx[k];
            System.out.printf("  state %2d: %.4f%n", i, bel[i]);
        }

        System.out.print("Truth trajectory: ");
        for (int i = 0; i < truth.length; i++) {
            System.out.print(truth[i]);
            System.out.print(i + 1 < truth.length ? ", " : "\n");
        }
    }
}
