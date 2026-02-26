// Chapter13_Lesson5.java
// ATE evaluation in Java using EJML for SVD (Umeyama alignment).
//
// Dependency (Gradle):
//   implementation "org.ejml:ejml-simple:0.43"
//
// Run:
//   java -cp ".;ejml-simple-0.43.jar;ejml-core-0.43.jar;ejml-ddense-0.43.jar" Chapter13_Lesson5 gt.txt est.txt 0.02 true
//
// Args:
//   gt tum file, est tum file, max_dt (s), allowScale (true/false)
import org.ejml.simple.SimpleMatrix;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;

public class Chapter13_Lesson5 {

    static class Traj {
        final double[] t;
        final double[][] p; // Nx3
        Traj(double[] t, double[][] p) { this.t = t; this.p = p; }
    }

    static Traj readTum(String path) throws Exception {
        List<Double> ts = new ArrayList<>();
        List<double[]> ps = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                String[] parts = line.split("\\s+");
                if (parts.length < 8) continue;
                double tt = Double.parseDouble(parts[0]);
                double tx = Double.parseDouble(parts[1]);
                double ty = Double.parseDouble(parts[2]);
                double tz = Double.parseDouble(parts[3]);
                ts.add(tt);
                ps.add(new double[]{tx, ty, tz});
            }
        }
        double[] t = new double[ts.size()];
        double[][] p = new double[ts.size()][3];
        for (int i = 0; i < ts.size(); i++) {
            t[i] = ts.get(i);
            p[i] = ps.get(i);
        }
        return new Traj(t, p);
    }

    static List<int[]> associateByTime(double[] ta, double[] tb, double maxDt) {
        List<int[]> pairs = new ArrayList<>();
        int j = 0;
        HashSet<Integer> used = new HashSet<>();
        for (int i = 0; i < ta.length; i++) {
            double t = ta[i];
            while (j + 1 < tb.length && tb[j] < t) j++;
            int[] cand = new int[]{j, j - 1};
            int best = -1;
            double bestErr = Double.POSITIVE_INFINITY;
            for (int jj : cand) {
                if (0 <= jj && jj < tb.length && !used.contains(jj)) {
                    double err = Math.abs(tb[jj] - t);
                    if (err < bestErr) { bestErr = err; best = jj; }
                }
            }
            if (best >= 0 && bestErr <= maxDt) {
                pairs.add(new int[]{i, best});
                used.add(best);
            }
        }
        return pairs;
    }

    static class Align {
        final double s;
        final SimpleMatrix R; // 3x3
        final SimpleMatrix t; // 3x1
        Align(double s, SimpleMatrix R, SimpleMatrix t) { this.s = s; this.R = R; this.t = t; }
    }

    static Align umeyama(SimpleMatrix A, SimpleMatrix B, boolean withScale) {
        int N = A.numCols();
        if (N < 3) throw new IllegalArgumentException("Need >=3 points");

        SimpleMatrix muA = A.mean(1);
        SimpleMatrix muB = B.mean(1);

        SimpleMatrix X = A.minus(muA.mult(SimpleMatrix.ones(1, N)));
        SimpleMatrix Y = B.minus(muB.mult(SimpleMatrix.ones(1, N)));

        SimpleMatrix Sigma = Y.mult(X.transpose()).divide(N);

        var svd = Sigma.svd();
        SimpleMatrix U = svd.getU();
        SimpleMatrix V = svd.getV();
        SimpleMatrix W = svd.getW(); // diagonal singular values

        SimpleMatrix S = SimpleMatrix.identity(3);
        if (U.determinant() * V.determinant() < 0.0) {
            S.set(2, 2, -1.0);
        }

        SimpleMatrix R = U.mult(S).mult(V.transpose());

        double s;
        if (withScale) {
            double varA = X.elementMult(X).elementSum() / N;
            double tr = 0.0;
            for (int i = 0; i < 3; i++) tr += W.get(i, i) * S.get(i, i);
            s = tr / (varA + 1e-12);
        } else {
            s = 1.0;
        }

        SimpleMatrix t = muB.minus(R.mult(muA).scale(s));
        return new Align(s, R, t);
    }

    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.US);
        if (args.length < 4) {
            System.err.println("Usage: Chapter13_Lesson5 gt.txt est.txt max_dt allowScale(true/false)");
            System.exit(2);
        }
        String gtPath = args[0];
        String estPath = args[1];
        double maxDt = Double.parseDouble(args[2]);
        boolean allowScale = Boolean.parseBoolean(args[3]);

        Traj gt = readTum(gtPath);
        Traj est = readTum(estPath);

        List<int[]> pairs = associateByTime(est.t, gt.t, maxDt);
        if (pairs.size() < 3) {
            System.err.println("Too few associated poses: " + pairs.size());
            System.exit(3);
        }

        int N = pairs.size();
        SimpleMatrix A = new SimpleMatrix(3, N);
        SimpleMatrix B = new SimpleMatrix(3, N);
        for (int k = 0; k < N; k++) {
            int i = pairs.get(k)[0];
            int j = pairs.get(k)[1];
            A.set(0, k, est.p[i][0]); A.set(1, k, est.p[i][1]); A.set(2, k, est.p[i][2]);
            B.set(0, k, gt.p[j][0]);  B.set(1, k, gt.p[j][1]);  B.set(2, k, gt.p[j][2]);
        }

        Align al = umeyama(A, B, allowScale);
        SimpleMatrix A_al = al.R.mult(A).scale(al.s).plus(al.t.mult(SimpleMatrix.ones(1, N)));

        double sum = 0.0, max = 0.0;
        double[] errs = new double[N];
        for (int k = 0; k < N; k++) {
            double dx = A_al.get(0,k) - B.get(0,k);
            double dy = A_al.get(1,k) - B.get(1,k);
            double dz = A_al.get(2,k) - B.get(2,k);
            double e = Math.sqrt(dx*dx + dy*dy + dz*dz);
            errs[k] = e;
            sum += e*e;
            if (e > max) max = e;
        }
        double rmse = Math.sqrt(sum / N);

        // median (quickselect-ish via sort for simplicity)
        java.util.Arrays.sort(errs);
        double median = errs[N/2];

        System.out.println("pairs " + N);
        System.out.println("scale " + al.s);
        System.out.println("rmse_m " + rmse);
        System.out.println("median_m " + median);
        System.out.println("max_m " + max);
    }
}
