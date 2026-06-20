// Chapter15_Lesson2.java
// Stanley Controller for Ground Vehicles (kinematic bicycle model, educational)
//
// Compile:
//   javac Chapter15_Lesson2.java
// Run:
//   java Chapter15_Lesson2

import java.util.Arrays;

public class Chapter15_Lesson2 {

    static double wrapAngle(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0 * pi);
        if (a < 0) a += 2.0 * pi;
        return a - pi;
    }

    static class State {
        double x, y, psi, v;
        State(double x, double y, double psi, double v) {
            this.x = x; this.y = y; this.psi = psi; this.v = v;
        }
    }

    static class Params {
        double L = 2.7;
        double k = 1.4;
        double v0 = 0.5;
        double maxSteer = Math.toRadians(30.0);
    }

    static void makeSPath(double[] xr, double[] yr) {
        int n = xr.length;
        double x0 = 0.0, x1 = 50.0;
        for (int i = 0; i < n; i++) {
            double x = x0 + (x1 - x0) * (double)i / (double)(n - 1);
            xr[i] = x;
            yr[i] = 2.5 * Math.sin(0.18 * x) + 1.0 * Math.sin(0.04 * x);
        }
    }

    static int nearestIndex(double px, double py, double[] xr, double[] yr) {
        double best = Double.POSITIVE_INFINITY;
        int bestI = 0;
        for (int i = 0; i < xr.length; i++) {
            double dx = px - xr[i];
            double dy = py - yr[i];
            double d2 = dx*dx + dy*dy;
            if (d2 < best) { best = d2; bestI = i; }
        }
        return bestI;
    }

    static double headingFromPath(int i, double[] xr, double[] yr) {
        int n = xr.length;
        int j = Math.min(i + 1, n - 1);
        int k = Math.max(i - 1, 0);
        double dx = xr[j] - xr[k];
        double dy = yr[j] - yr[k];
        return Math.atan2(dy, dx);
    }

    static double stanleyControl(State s, double[] xr, double[] yr, Params p, double[] outErr) {
        int i = nearestIndex(s.x, s.y, xr, yr);
        double psiRef = headingFromPath(i, xr, yr);

        // Convention: ePsi = psiRef - psi
        double ePsi = wrapAngle(psiRef - s.psi);

        // Signed lateral error via 2D cross product
        double tx = Math.cos(psiRef), ty = Math.sin(psiRef);
        double vx = s.x - xr[i], vy = s.y - yr[i];
        double cross = tx*vy - ty*vx;
        double eY = Math.copySign(Math.hypot(vx, vy), cross);

        double delta = ePsi + Math.atan2(p.k * eY, s.v + p.v0);
        delta = Math.max(-p.maxSteer, Math.min(p.maxSteer, delta));

        outErr[0] = eY;
        outErr[1] = ePsi;
        return delta;
    }

    static State stepBicycle(State s, double delta, double dt, double L) {
        double x = s.x + s.v * Math.cos(s.psi) * dt;
        double y = s.y + s.v * Math.sin(s.psi) * dt;
        double psi = wrapAngle(s.psi + s.v / L * Math.tan(delta) * dt);
        return new State(x, y, psi, s.v);
    }

    public static void main(String[] args) {
        int n = 400;
        double[] xr = new double[n];
        double[] yr = new double[n];
        makeSPath(xr, yr);

        Params p = new Params();
        double dt = 0.02, T = 25.0;
        int steps = (int)(T / dt);

        State s = new State(-2.0, 3.5, Math.toRadians(-10.0), 6.0);

        double sumSq = 0.0;
        double[] err = new double[2];
        for (int k = 0; k < steps; k++) {
            double delta = stanleyControl(s, xr, yr, p, err);
            s = stepBicycle(s, delta, dt, p.L);
            sumSq += err[0] * err[0];
        }

        double rms = Math.sqrt(sumSq / steps);
        System.out.println("Done. RMS cross-track error eY: " + rms + " m");
        System.out.println("Final state: x=" + s.x + ", y=" + s.y + ", psi=" + s.psi);
    }
}
