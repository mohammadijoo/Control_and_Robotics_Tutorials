// Chapter19_Lesson3.java
// Standard Datasets and Simulated Benchmarks for AMR
// Pure Java reference implementation for simple trajectory metric computation.
//
// CSV format: t,x,y,yaw
// Compile: javac Chapter19_Lesson3.java
// Run:     java Chapter19_Lesson3 gt.csv est.csv

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Chapter19_Lesson3 {

    static class TrajectorySE2 {
        List<Double> t = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        List<Double> yaw = new ArrayList<>();
    }

    static class Pose {
        double x, y, yaw;
        Pose(double x, double y, double yaw) {
            this.x = x; this.y = y; this.yaw = yaw;
        }
    }

    static class Metrics {
        double ateRmse;
        double rpeRmse;
        double rpeYawMean;
        int pairs;
    }

    static TrajectorySE2 loadCsv(String path) throws IOException {
        TrajectorySE2 tr = new TrajectorySE2();
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line = br.readLine(); // header
            if (line == null) throw new IOException("Empty file");
            while ((line = br.readLine()) != null) {
                if (line.trim().isEmpty()) continue;
                String[] parts = line.split(",");
                if (parts.length != 4) throw new IOException("Expected 4 columns in " + path);
                tr.t.add(Double.parseDouble(parts[0].trim()));
                tr.x.add(Double.parseDouble(parts[1].trim()));
                tr.y.add(Double.parseDouble(parts[2].trim()));
                tr.yaw.add(Double.parseDouble(parts[3].trim()));
            }
        }
        return tr;
    }

    static double wrapAngle(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    static int findSegment(List<Double> t, double tq) {
        if (tq < t.get(0) || tq > t.get(t.size()-1)) {
            throw new IllegalArgumentException("query time out of bounds");
        }
        int j = Collections.binarySearch(t, tq);
        if (j >= 0) {
            if (j == t.size() - 1) return j - 1;
            return j;
        }
        j = -j - 1;
        return Math.max(0, j - 1);
    }

    static Pose interpPose(TrajectorySE2 tr, double tq) {
        int i = findSegment(tr.t, tq);
        int j = i + 1;
        double ti = tr.t.get(i), tj = tr.t.get(j);
        double a = (tq - ti) / (tj - ti);

        double x = (1.0 - a) * tr.x.get(i) + a * tr.x.get(j);
        double y = (1.0 - a) * tr.y.get(i) + a * tr.y.get(j);

        double c = (1.0 - a) * Math.cos(tr.yaw.get(i)) + a * Math.cos(tr.yaw.get(j));
        double s = (1.0 - a) * Math.sin(tr.yaw.get(i)) + a * Math.sin(tr.yaw.get(j));
        double yaw = Math.atan2(s, c);
        return new Pose(x, y, yaw);
    }

    static double[] relativeSE2(Pose p0, Pose p1) {
        double dxw = p1.x - p0.x;
        double dyw = p1.y - p0.y;
        double c = Math.cos(p0.yaw), s = Math.sin(p0.yaw);
        double dx = c * dxw + s * dyw;
        double dy = -s * dxw + c * dyw;
        double dyaw = wrapAngle(p1.yaw - p0.yaw);
        return new double[]{dx, dy, dyaw};
    }

    static Metrics evaluate(TrajectorySE2 gt, TrajectorySE2 est, double deltaT) {
        List<Pose> gtInterp = new ArrayList<>();
        List<Pose> estPose = new ArrayList<>();

        for (int i = 0; i < est.t.size(); i++) {
            Pose g = interpPose(gt, est.t.get(i));
            Pose e = new Pose(est.x.get(i), est.y.get(i), est.yaw.get(i));
            gtInterp.add(g);
            estPose.add(e);
        }

        // For simplicity, no global rigid alignment in this Java example (compare in common frame)
        double sse = 0.0;
        for (int i = 0; i < estPose.size(); i++) {
            double ex = estPose.get(i).x - gtInterp.get(i).x;
            double ey = estPose.get(i).y - gtInterp.get(i).y;
            sse += ex*ex + ey*ey;
        }

        Metrics m = new Metrics();
        m.ateRmse = Math.sqrt(sse / estPose.size());

        double sseRpe = 0.0;
        double sumYaw = 0.0;
        int pairs = 0;
        for (int i = 0; i < est.t.size(); i++) {
            double target = est.t.get(i) + deltaT;
            int j = -1;
            for (int k = i + 1; k < est.t.size(); k++) {
                if (est.t.get(k) >= target) { j = k; break; }
            }
            if (j < 0) continue;

            double[] dgt = relativeSE2(gtInterp.get(i), gtInterp.get(j));
            double[] dest = relativeSE2(estPose.get(i), estPose.get(j));

            double dx = dest[0] - dgt[0];
            double dy = dest[1] - dgt[1];
            double dyaw = wrapAngle(dest[2] - dgt[2]);

            sseRpe += dx*dx + dy*dy;
            sumYaw += Math.abs(dyaw);
            pairs++;
        }

        m.pairs = pairs;
        if (pairs > 0) {
            m.rpeRmse = Math.sqrt(sseRpe / pairs);
            m.rpeYawMean = sumYaw / pairs;
        }
        return m;
    }

    public static void main(String[] args) {
        if (args.length < 2) {
            System.err.println("Usage: java Chapter19_Lesson3 gt.csv est.csv [delta_t]");
            System.exit(1);
        }
        try {
            double deltaT = (args.length >= 3) ? Double.parseDouble(args[2]) : 1.0;
            TrajectorySE2 gt = loadCsv(args[0]);
            TrajectorySE2 est = loadCsv(args[1]);
            Metrics m = evaluate(gt, est, deltaT);

            System.out.printf("ATE_trans_rmse_m = %.6f%n", m.ateRmse);
            System.out.printf("RPE_trans_rmse_m = %.6f%n", m.rpeRmse);
            System.out.printf("RPE_yaw_mean_rad = %.6f%n", m.rpeYawMean);
            System.out.printf("Pairs = %d%n", m.pairs);

            // Example normalized score (lower-is-better metrics)
            double capATE = 2.0, capRPE = 0.8;
            double sATE = Math.max(0.0, 1.0 - m.ateRmse / capATE);
            double sRPE = Math.max(0.0, 1.0 - m.rpeRmse / capRPE);
            double score = 100.0 * 0.5 * (sATE + sRPE);
            System.out.printf("Normalized score = %.2f/100%n", score);

        } catch (Exception ex) {
            ex.printStackTrace();
            System.exit(2);
        }
    }
}
