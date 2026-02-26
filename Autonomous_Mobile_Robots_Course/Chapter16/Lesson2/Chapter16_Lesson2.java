// Chapter16_Lesson2.java
// Autonomous Mobile Robots — Chapter 16, Lesson 2: Velocity Obstacles (VO) Sampling Demo
//
// Educational reference implementation (minimal):
// - Time-horizon collision check in relative motion
// - Sampling-based safe velocity selection
//
// Compile/run:
//   javac Chapter16_Lesson2.java
//   java Chapter16_Lesson2

import java.util.*;

public class Chapter16_Lesson2 {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 add(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
        Vec2 sub(Vec2 o) { return new Vec2(x - o.x, y - o.y); }
        Vec2 mul(double s) { return new Vec2(x * s, y * s); }
    }

    static double dot(Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
    static double norm2(Vec2 a) { return dot(a,a); }
    static double norm(Vec2 a) { return Math.sqrt(norm2(a)); }

    static Vec2 clampNorm(Vec2 v, double vmax) {
        double n = norm(v);
        if (n <= vmax) return v;
        double s = vmax / (n + 1e-12);
        return v.mul(s);
    }

    static class Agent {
        Vec2 p, v, goal;
        double radius = 0.35;
        double vmax = 1.2;
        Agent(Vec2 p, Vec2 v, Vec2 goal) {
            this.p = p; this.v = v; this.goal = goal;
        }
    }

    static class TTC {
        boolean collides;
        double tHit;
        TTC(boolean c, double t) { collides=c; tHit=t; }
    }

    static TTC timeToCollisionInHorizon(Vec2 pRel, Vec2 vRel, double R, double T) {
        if (norm(pRel) <= R) return new TTC(true, 0.0);

        double a = norm2(vRel);
        double b = 2.0 * dot(pRel, vRel);
        double c = norm2(pRel) - R*R;

        if (a < 1e-12) return new TTC(false, Double.POSITIVE_INFINITY);

        double disc = b*b - 4.0*a*c;
        if (disc < 0.0) return new TTC(false, Double.POSITIVE_INFINITY);

        double sdisc = Math.sqrt(disc);
        double t1 = (-b - sdisc) / (2.0*a);
        double t2 = (-b + sdisc) / (2.0*a);

        double tHit = Double.POSITIVE_INFINITY;
        if (t1 >= 0.0) tHit = t1;
        else if (t2 >= 0.0) tHit = 0.0;

        if (tHit >= 0.0 && tHit <= T) return new TTC(true, tHit);
        return new TTC(false, Double.POSITIVE_INFINITY);
    }

    static List<Vec2> sampleDisk(double vmax, int n, Random rng) {
        ArrayList<Vec2> out = new ArrayList<>(n);
        for (int i=0;i<n;i++) {
            double ang = 2.0*Math.PI*rng.nextDouble();
            double r = Math.sqrt(rng.nextDouble())*vmax;
            out.add(new Vec2(r*Math.cos(ang), r*Math.sin(ang)));
        }
        return out;
    }

    static Vec2 chooseVelocityVO(Agent a, List<Agent> neighbors, double dt, double T, Random rng) {
        Vec2 toGoal = a.goal.sub(a.p);
        double d = norm(toGoal);
        Vec2 vPref = new Vec2(0.0, 0.0);
        if (d > 1e-9) {
            double spd = Math.min(a.vmax, d / Math.max(dt, 1e-3));
            vPref = toGoal.mul(spd / (d + 1e-12));
        }

        List<Vec2> cand = sampleDisk(a.vmax, 1200, rng);
        cand.add(clampNorm(vPref, a.vmax));

        Vec2 best = new Vec2(0.0, 0.0);
        double bestCost = Double.POSITIVE_INFINITY;

        for (Vec2 v : cand) {
            boolean feasible = true;
            double minTTC = Double.POSITIVE_INFINITY;

            for (Agent nb : neighbors) {
                Vec2 pRel = nb.p.sub(a.p);
                Vec2 vRel = v.sub(nb.v);
                double R = a.radius + nb.radius;

                TTC res = timeToCollisionInHorizon(pRel, vRel, R, T);
                if (res.collides) { feasible=false; break; }

                TTC resAny = timeToCollisionInHorizon(pRel, vRel, R, 1e6);
                if (resAny.collides) minTTC = Math.min(minTTC, resAny.tHit);
            }

            if (!feasible) continue;

            double prefCost = norm2(v.sub(vPref));
            double safetyCost = 0.0;
            if (Double.isFinite(minTTC)) safetyCost = 1.0 / (minTTC + 1e-6);

            double cost = 1.0*prefCost + 2.0*safetyCost;
            if (cost < bestCost) { bestCost = cost; best = v; }
        }

        if (!Double.isFinite(bestCost)) return a.v.mul(0.2);
        return best;
    }

    public static void main(String[] args) {
        Random rng = new Random(1);

        final int N = 8;
        final int steps = 400;
        final double dt = 0.05;
        final double T = 2.5;
        final double R0 = 5.0;

        ArrayList<Agent> agents = new ArrayList<>(N);
        for (int i=0;i<N;i++) {
            double ang = 2.0*Math.PI*i/N;
            Vec2 p = new Vec2(R0*Math.cos(ang), R0*Math.sin(ang));
            Vec2 goal = new Vec2(-p.x, -p.y);
            agents.add(new Agent(p, new Vec2(0.0,0.0), goal));
        }

        for (int k=0;k<steps;k++) {
            Vec2[] nextV = new Vec2[N];

            for (int i=0;i<N;i++) {
                ArrayList<Agent> neigh = new ArrayList<>(N-1);
                for (int j=0;j<N;j++) if (j!=i) neigh.add(agents.get(j));
                nextV[i] = chooseVelocityVO(agents.get(i), neigh, dt, T, rng);
            }

            for (int i=0;i<N;i++) {
                Agent a = agents.get(i);
                a.v = nextV[i];
                a.p = a.p.add(a.v.mul(dt));
            }

            if (k % 50 == 0) {
                Agent a0 = agents.get(0);
                System.out.printf("Step %d: agent0 p=(%.3f, %.3f)%n", k, a0.p.x, a0.p.y);
            }
        }
        System.out.println("Done.");
    }
}
