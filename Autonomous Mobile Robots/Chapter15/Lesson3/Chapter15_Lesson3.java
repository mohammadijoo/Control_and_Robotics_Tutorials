/*
Chapter15_Lesson3.java
Dynamic Window Approach (DWA) — minimal Java implementation (no ROS required).
Compile:
  javac Chapter15_Lesson3.java
Run:
  java Chapter15_Lesson3
*/

import java.util.ArrayList;
import java.util.List;

public class Chapter15_Lesson3 {

    static class State {
        double x, y, theta, v, w;
        State(double x, double y, double theta, double v, double w) {
            this.x = x; this.y = y; this.theta = theta; this.v = v; this.w = w;
        }
    }

    static class Config {
        double vMin = -0.2, vMax = 1.0;
        double wMin = -1.5, wMax = 1.5;

        double aMax = 0.8;
        double aBrake = 1.0;
        double alphaMax = 2.0;

        double dt = 0.1;
        double T = 2.0;
        double vRes = 0.05;
        double wRes = 0.1;

        double robotRadius = 0.3;

        double wHeading = 0.4;
        double wClear = 0.4;
        double wSpeed = 0.2;
    }

    static class Candidate {
        double h, c, s, v, w;
        List<State> traj;
        Candidate(double h, double c, double s, double v, double w, List<State> traj) {
            this.h = h; this.c = c; this.s = s; this.v = v; this.w = w; this.traj = traj;
        }
    }

    static double wrapToPi(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0 * pi);
        if (a < 0) a += 2.0 * pi;
        return a - pi;
    }

    static State motionStep(State s, double v, double w, double dt) {
        double nx = s.x + v * Math.cos(s.theta) * dt;
        double ny = s.y + v * Math.sin(s.theta) * dt;
        double nt = s.theta + w * dt;
        return new State(nx, ny, nt, v, w);
    }

    static double[] dynamicWindow(State s, Config cfg) {
        double vLowDyn = s.v - cfg.aBrake * cfg.dt;
        double vHighDyn = s.v + cfg.aMax * cfg.dt;
        double wLowDyn = s.w - cfg.alphaMax * cfg.dt;
        double wHighDyn = s.w + cfg.alphaMax * cfg.dt;

        double vL = Math.max(cfg.vMin, vLowDyn);
        double vU = Math.min(cfg.vMax, vHighDyn);
        double wL = Math.max(cfg.wMin, wLowDyn);
        double wU = Math.min(cfg.wMax, wHighDyn);
        return new double[]{vL, vU, wL, wU};
    }

    static List<State> rollout(State s0, double v, double w, Config cfg) {
        int n = (int)Math.floor(cfg.T / cfg.dt);
        List<State> traj = new ArrayList<>();
        traj.add(s0);
        State s = s0;
        for (int i = 0; i < n; i++) {
            s = motionStep(s, v, w, cfg.dt);
            traj.add(s);
        }
        return traj;
    }

    static double minClearance(List<State> traj, double[][] obs) {
        double best = Double.POSITIVE_INFINITY;
        for (State st : traj) {
            for (double[] o : obs) {
                double dx = st.x - o[0];
                double dy = st.y - o[1];
                double d = Math.hypot(dx, dy);
                if (d < best) best = d;
            }
        }
        return best;
    }

    static double headingScore(List<State> traj, double gx, double gy) {
        State last = traj.get(traj.size() - 1);
        double dir = Math.atan2(gy - last.y, gx - last.x);
        double err = wrapToPi(dir - last.theta);
        return Math.cos(err);
    }

    static double speedScore(double v, Config cfg) {
        return (v - cfg.vMin) / Math.max(1e-9, (cfg.vMax - cfg.vMin));
    }

    static boolean admissible(double v, double clear, Config cfg) {
        double vv = Math.max(0.0, v);
        double dStop = (vv * vv) / Math.max(1e-9, 2.0 * cfg.aBrake);
        return clear > (dStop + cfg.robotRadius);
    }

    static double[] normalize(double[] vals) {
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (double v : vals) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
        double[] out = new double[vals.length];
        if (Math.abs(hi - lo) < 1e-12) return out; // all zeros
        for (int i = 0; i < vals.length; i++) out[i] = (vals[i] - lo) / (hi - lo);
        return out;
    }

    static Object[] dwaControl(State s, double gx, double gy, double[][] obs, Config cfg) {
        double[] win = dynamicWindow(s, cfg);
        double vL = win[0], vU = win[1], wL = win[2], wU = win[3];

        List<Candidate> cand = new ArrayList<>();
        for (double v = vL; v <= vU + 1e-12; v += cfg.vRes) {
            for (double w = wL; w <= wU + 1e-12; w += cfg.wRes) {
                List<State> traj = rollout(s, v, w, cfg);
                double clear = minClearance(traj, obs) - cfg.robotRadius;
                if (!admissible(v, clear, cfg)) continue;
                double h = headingScore(traj, gx, gy);
                double sp = speedScore(v, cfg);
                cand.add(new Candidate(h, clear, sp, v, w, traj));
            }
        }

        if (cand.isEmpty()) return new Object[]{0.0, 0.0, List.of(s)};

        double[] hs = new double[cand.size()];
        double[] cs = new double[cand.size()];
        double[] ss = new double<cand.size()];
        for (int i = 0; i < cand.size(); i++) {
            hs[i] = cand.get(i).h;
            cs[i] = cand.get(i).c;
            ss[i] = cand.get(i).s;
        }
        hs = normalize(hs);
        cs = normalize(cs);
        ss = normalize(ss);

        double bestJ = -1e18;
        int bestIdx = 0;
        for (int i = 0; i < cand.size(); i++) {
            double J = cfg.wHeading * hs[i] + cfg.wClear * cs[i] + cfg.wSpeed * ss[i];
            if (J > bestJ) { bestJ = J; bestIdx = i; }
        }
        Candidate best = cand.get(bestIdx);
        return new Object[]{best.v, best.w, best.traj};
    }

    public static void main(String[] args) {
        Config cfg = new Config();
        State s = new State(0.0, 0.0, 0.0, 0.0, 0.0);
        double gx = 8.0, gy = 6.0;

        List<double[]> obsList = new ArrayList<>();
        for (int i = 0; i < 15; i++) obsList.add(new double[]{2.0 + i*(5.0/14.0), 3.0});
        for (int i = 0; i < 12; i++) obsList.add(new double[]{4.0, 1.0 + i*(4.0/11.0)});
        double[][] obs = obsList.toArray(new double[0][0]);

        for (int k = 0; k < 600; k++) {
            Object[] out = dwaControl(s, gx, gy, obs, cfg);
            double v = (double)out[0];
            double w = (double)out[1];
            s = motionStep(s, v, w, cfg.dt);
            double dist = Math.hypot(s.x - gx, s.y - gy);
            if (k % 20 == 0) {
                System.out.printf("k=%d x=%.3f y=%.3f dist=%.3f v=%.2f w=%.2f%n", k, s.x, s.y, dist, v, w);
            }
            if (dist < 0.3) break;
        }
        System.out.printf("Final: x=%.3f y=%.3f theta=%.3f%n", s.x, s.y, s.theta);
    }
}
