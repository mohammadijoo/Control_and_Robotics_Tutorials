// Chapter19_Lesson2.java
/*
Metrics for Navigation Robustness (Autonomous Mobile Robots)

Compile:
  javac Chapter19_Lesson2.java
Run:
  java Chapter19_Lesson2

This Java example creates a synthetic mission log and computes a core set of robustness metrics.
Production integration notes:
- ROSJava / rosbridge clients can stream odometry, goal state, and event flags into CSV or DB.
- Apache Commons Math can be added for advanced statistics (bootstrap, hypothesis tests).
*/

import java.util.*;
import java.util.stream.*;

public class Chapter19_Lesson2 {

    static class Sample {
        int episodeId;
        double timeS;
        double x, y;
        double goalX, goalY;
        double clearanceM;
        int collision;
        int intervention;
        int goalReached;
        double trackingErrorM;
        double cmdV, cmdVMax;
        double cmdW, cmdWMax;
        int recoveryEvent;
    }

    static class EpisodeMetrics {
        int episodeId;
        int success;
        int collisionFree;
        int interventionFree;
        int hadFailure;
        int recoveredAfterFailure;
        double completionTimeS;
        double pathLengthM;
        double referenceDistanceM;
        double pathEfficiency;
        double minClearanceM;
        double trackingRmseM;
        double saturationRatio;
    }

    static class Wilson {
        double pHat, low, high;
        Wilson(double pHat, double low, double high) {
            this.pHat = pHat; this.low = low; this.high = high;
        }
    }

    static Wilson wilsonInterval(int k, int n, double z) {
        if (n <= 0) return new Wilson(Double.NaN, Double.NaN, Double.NaN);
        double p = (double) k / n;
        double denom = 1.0 + (z * z) / n;
        double center = (p + (z * z) / (2.0 * n)) / denom;
        double rad = (z / denom) * Math.sqrt((p * (1.0 - p) / n) + (z * z) / (4.0 * n * n));
        return new Wilson(p, center - rad, center + rad);
    }

    static double dist(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1, dy = y2 - y1;
        return Math.hypot(dx, dy);
    }

    static List<Sample> syntheticData() {
        List<Sample> list = new ArrayList<>();
        Random rng = new Random(19);
        for (int ep = 0; ep < 10; ep++) {
            double x = 0.0, y = 0.0;
            double gx = 9.0 + 0.3 * ep;
            double gy = (ep % 4) - 1.5;
            boolean stop = false;
            for (int k = 0; k < 180 && !stop; k++) {
                double t = 0.1 * k;
                double d = dist(x, y, gx, gy);
                double theta = Math.atan2(gy - y, gx - x);
                double vMax = 0.8, wMax = 1.2;
                double v = Math.max(-vMax, Math.min(vMax, 0.55 + 0.04 * Math.sin(0.07 * k + ep)));
                double w = Math.max(-wMax, Math.min(wMax, 0.30 * Math.sin(0.06 * k)));
                x += 0.1 * v * Math.cos(theta);
                y += 0.1 * v * Math.sin(theta);

                double clearance = 0.70 + 0.20 * Math.sin(0.09 * k + ep) + 0.02 * (rng.nextDouble() - 0.5);
                if (ep == 3 && k > 70 && k < 85) clearance -= 0.58;
                if (ep == 7 && k > 40 && k < 54) clearance -= 0.47;
                clearance = Math.max(0.0, clearance);

                int collision = (clearance < 0.05) ? 1 : 0;
                int intervention = (clearance < 0.12 && collision == 0) ? 1 : 0;
                int goalReached = (d < 0.33 && collision == 0) ? 1 : 0;
                int recovery = (intervention == 1 && (k % 6 == 0)) ? 1 : 0;
                double eTrack = Math.abs(0.06 * Math.sin(0.025 * k + 0.5 * ep));

                Sample s = new Sample();
                s.episodeId = ep; s.timeS = t;
                s.x = x; s.y = y;
                s.goalX = gx; s.goalY = gy;
                s.clearanceM = clearance;
                s.collision = collision; s.intervention = intervention;
                s.goalReached = goalReached;
                s.trackingErrorM = eTrack;
                s.cmdV = v; s.cmdVMax = vMax;
                s.cmdW = w; s.cmdWMax = wMax;
                s.recoveryEvent = recovery;
                list.add(s);

                if (collision == 1 || goalReached == 1) stop = true;
            }
        }
        return list;
    }

    static List<EpisodeMetrics> evaluate(List<Sample> data) {
        Map<Integer, List<Sample>> grouped = new TreeMap<>();
        for (Sample s : data) grouped.computeIfAbsent(s.episodeId, k -> new ArrayList<>()).add(s);

        List<EpisodeMetrics> out = new ArrayList<>();
        for (Map.Entry<Integer, List<Sample>> e : grouped.entrySet()) {
            List<Sample> g = e.getValue();
            g.sort(Comparator.comparingDouble(a -> a.timeS));

            EpisodeMetrics m = new EpisodeMetrics();
            m.episodeId = e.getKey();
            m.minClearanceM = Double.POSITIVE_INFINITY;
            m.collisionFree = 1;
            m.interventionFree = 1;

            int satCount = 0;
            double sumE2 = 0.0;
            for (int i = 0; i < g.size(); i++) {
                Sample s = g.get(i);
                m.success = Math.max(m.success, s.goalReached);
                                if (s.collision == 1) m.collisionFree = 0;
                if (s.intervention == 1) m.interventionFree = 0;
                if (s.collision + s.intervention > 0) m.hadFailure = 1;
                if (s.recoveryEvent == 1) m.recoveredAfterFailure = 1;

                m.minClearanceM = Math.min(m.minClearanceM, s.clearanceM);
                sumE2 += s.trackingErrorM * s.trackingErrorM;

                boolean satV = Math.abs(s.cmdV) >= 0.98 * Math.max(s.cmdVMax, 1e-9);
                boolean satW = Math.abs(s.cmdW) >= 0.98 * Math.max(s.cmdWMax, 1e-9);
                if (satV || satW) satCount++;

                if (i > 0) {
                    Sample p = g.get(i - 1);
                    m.pathLengthM += dist(p.x, p.y, s.x, s.y);
                }
            }
            Sample first = g.get(0), last = g.get(g.size() - 1);
            m.completionTimeS = last.timeS - first.timeS;
            m.referenceDistanceM = dist(first.x, first.y, first.goalX, first.goalY);
            m.pathEfficiency = (m.success == 1) ? Math.min(1.0, m.referenceDistanceM / Math.max(m.pathLengthM, 1e-9)) : 0.0;
            m.trackingRmseM = Math.sqrt(sumE2 / Math.max(1, g.size()));
            m.saturationRatio = (double) satCount / Math.max(1, g.size());
            if (!(m.hadFailure == 1 && m.recoveredAfterFailure == 1 && m.success == 1)) {
                m.recoveredAfterFailure = 0;
            }
            out.add(m);
        }
        return out;
    }

    public static void main(String[] args) {
        List<Sample> data = syntheticData();
        List<EpisodeMetrics> metrics = evaluate(data);

        int n = metrics.size();
        int kSuccess = 0, kCollisionFree = 0, kInterventionFree = 0, kRecovered = 0, nFailed = 0;
        double meanEta = 0, meanClear = 0, meanTrack = 0, meanTimeSucc = 0;
        int nSucc = 0;

        for (EpisodeMetrics m : metrics) {
            kSuccess += m.success;
            kCollisionFree += m.collisionFree;
            kInterventionFree += m.interventionFree;
            kRecovered += m.recoveredAfterFailure;
            nFailed += m.hadFailure;
            meanEta += m.pathEfficiency;
            meanClear += m.minClearanceM;
            meanTrack += m.trackingRmseM;
            if (m.success == 1) { meanTimeSucc += m.completionTimeS; nSucc++; }
        }

        meanEta /= Math.max(1, n);
        meanClear /= Math.max(1, n);
        meanTrack /= Math.max(1, n);
        meanTimeSucc = (nSucc > 0) ? meanTimeSucc / nSucc : Double.NaN;

        Wilson wSuccess = wilsonInterval(kSuccess, n, 1.96);
        Wilson wSafe = wilsonInterval(kCollisionFree, n, 1.96);

        double successRate = (double) kSuccess / Math.max(1, n);
        double collisionFreeRate = (double) kCollisionFree / Math.max(1, n);
        double interventionFreeRate = (double) kInterventionFree / Math.max(1, n);

        double timeScore = 0, trackScore = 0, clearScore = 0;
        for (EpisodeMetrics m : metrics) {
            timeScore += Math.exp(-m.completionTimeS / 60.0) * m.success;
            trackScore += Math.exp(-m.trackingRmseM / 0.25);
            clearScore += Math.min(1.0, Math.max(0.0, m.minClearanceM / 0.30));
        }
        timeScore /= Math.max(1, n);
        trackScore /= Math.max(1, n);
        clearScore /= Math.max(1, n);

        double robustness =
            0.30 * collisionFreeRate +
            0.15 * interventionFreeRate +
            0.20 * successRate +
            0.15 * clearScore +
            0.10 * meanEta +
            0.05 * trackScore +
            0.05 * timeScore;

        System.out.println("Per-episode metrics:");
        for (EpisodeMetrics m : metrics) {
            System.out.printf(
                Locale.US,
                "ep=%d success=%d collisionFree=%d interventionFree=%d T=%.2f L=%.2f eta=%.3f minClear=%.3f trackRMSE=%.3f sat=%.3f%n",
                m.episodeId, m.success, m.collisionFree, m.interventionFree, m.completionTimeS, m.pathLengthM,
                m.pathEfficiency, m.minClearanceM, m.trackingRmseM, m.saturationRatio
            );
        }

        System.out.println("\nSummary metrics:");
        System.out.printf(Locale.US, "Success rate: %.4f (Wilson95%% [%.4f, %.4f])%n", wSuccess.pHat, wSuccess.low, wSuccess.high);
        System.out.printf(Locale.US, "Collision-free rate: %.4f (Wilson95%% [%.4f, %.4f])%n", wSafe.pHat, wSafe.low, wSafe.high);
        System.out.printf(Locale.US, "Intervention-free rate: %.4f%n", interventionFreeRate);
        System.out.printf(Locale.US, "Mean completion time (success): %.4f s%n", meanTimeSucc);
        System.out.printf(Locale.US, "Mean path efficiency: %.4f%n", meanEta);
        System.out.printf(Locale.US, "Mean min clearance: %.4f m%n", meanClear);
        System.out.printf(Locale.US, "Mean tracking RMSE: %.4f m%n", meanTrack);
        System.out.printf(Locale.US, "Recovery-after-failure rate: %.4f%n", (nFailed > 0 ? (double) kRecovered / nFailed : Double.NaN));
        System.out.printf(Locale.US, "Composite robustness score: %.4f%n", robustness);
    }
}
