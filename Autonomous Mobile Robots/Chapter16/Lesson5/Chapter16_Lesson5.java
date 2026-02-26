// Chapter16_Lesson5.java
// Autonomous Mobile Robots — Chapter 16, Lesson 5
// Lab: Navigate Through Moving Crowds
//
// Minimal Java simulator (no external deps):
//   - Crowd: constant-velocity discs with small jitter
//   - Robot: unicycle
//   - Planner: sampling-based crowd-aware DWA (TTC + social discomfort)
//
// Compile:
//   javac Chapter16_Lesson5.java
// Run:
//   java Chapter16_Lesson5
//
// Output: robot trajectory to "robot_traj_java.csv"

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter16_Lesson5 {

  static class Vec2 {
    double x, y;
    Vec2(double x, double y) { this.x = x; this.y = y; }
    Vec2 add(Vec2 o) { return new Vec2(this.x + o.x, this.y + o.y); }
    Vec2 sub(Vec2 o) { return new Vec2(this.x - o.x, this.y - o.y); }
    Vec2 mul(double s) { return new Vec2(this.x * s, this.y * s); }
  }

  static double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
  static double norm(Vec2 a) { return Math.sqrt(dot(a, a)); }

  static double wrapAngle(double a) {
    double pi = Math.PI;
    a = (a + pi) % (2.0 * pi);
    if (a < 0.0) a += 2.0 * pi;
    return a - pi;
  }

  static double clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  static class DiscAgent {
    Vec2 p, v;
    double r;
    DiscAgent(Vec2 p, Vec2 v, double r) { this.p = p; this.v = v; this.r = r; }
  }

  static class Robot {
    Vec2 p;
    double th, v, w, r;
    Robot(Vec2 p, double th, double v, double w, double r) {
      this.p = p; this.th = th; this.v = v; this.w = w; this.r = r;
    }
  }

  static class Params {
    double dt = 0.1;
    double horizon = 2.5;
    double vMax = 1.2;
    double wMax = 1.8;
    double aV = 1.0;
    double aW = 2.5;

    double wGoal = 5.0;
    double wClear = 2.0;
    double wTtc = 4.0;
    double wSoc = 2.0;
    double wVel = 0.5;
    double wTurn = 0.1;
    double wSmooth = 0.2;

    double sigmaFront = 0.9;
    double sigmaSide = 0.6;
    double sigmaBack = 0.5;
  }

  static double ttcDiscs(Vec2 pRel, Vec2 vRel, double R) {
    double eps = 1e-9;
    double a = dot(vRel, vRel);
    double b = 2.0 * dot(pRel, vRel);
    double c = dot(pRel, pRel) - R * R;

    if (c <= 0.0) return 0.0;
    if (a <= eps) return Double.POSITIVE_INFINITY;

    double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return Double.POSITIVE_INFINITY;

    double s = Math.sqrt(disc);
    double t1 = (-b - s) / (2.0 * a);
    double t2 = (-b + s) / (2.0 * a);

    if (t1 >= 0.0) return t1;
    if (t2 >= 0.0) return t2;
    return Double.POSITIVE_INFINITY;
  }

  static double socialCost(Vec2 rp, List<DiscAgent> humans, Params P) {
    double c = 0.0;
    for (DiscAgent h : humans) {
      double vnorm = norm(h.v);
      double phi = (vnorm < 1e-6) ? 0.0 : Math.atan2(h.v.y, h.v.x);

      // world->human frame is rotation by -phi
      double cphi = Math.cos(-phi), sphi = Math.sin(-phi);
      Vec2 rel = rp.sub(h.p);
      double dx = cphi * rel.x - sphi * rel.y;
      double dy = sphi * rel.x + cphi * rel.y;

      double sx = (dx >= 0.0) ? P.sigmaFront : P.sigmaBack;
      double sy = P.sigmaSide;
      double q = 0.5 * ((dx / sx) * (dx / sx) + (dy / sy) * (dy / sy));
      c += Math.exp(-q);
    }
    return c;
  }

  static void simulateUnicycle(Robot r, double v, double w, double dt) {
    r.p = new Vec2(
        r.p.x + v * Math.cos(r.th) * dt,
        r.p.y + v * Math.sin(r.th) * dt
    );
    r.th = wrapAngle(r.th + w * dt);
  }

  static List<DiscAgent> makeCrossingCrowd(int nPerStream, double speed, long seed) {
    Random rng = new Random(seed);
    List<DiscAgent> humans = new ArrayList<>();
    double rad = 0.25;

    // left->right
    for (int i = 0; i < nPerStream; i++) {
      double y = -2.0 + 4.0 * rng.nextDouble();
      double x = -7.0 + 3.5 * rng.nextDouble();
      humans.add(new DiscAgent(new Vec2(x, y), new Vec2(speed, 0.0), rad));
    }
    // bottom->top
    for (int i = 0; i < nPerStream; i++) {
      double x = -2.0 + 4.0 * rng.nextDouble();
      double y = -7.0 + 3.5 * rng.nextDouble();
      humans.add(new DiscAgent(new Vec2(x, y), new Vec2(0.0, speed), rad));
    }
    return humans;
  }

  static void updateHumans(List<DiscAgent> humans, double dt, double arena, double noiseStd, long seed) {
    Random rng = new Random(seed);
    for (DiscAgent h : humans) {
      if (noiseStd > 0.0) {
        h.v = new Vec2(h.v.x + noiseStd * rng.nextGaussian(), h.v.y + noiseStd * rng.nextGaussian());
        double sp = norm(h.v);
        if (sp > 1e-6) {
          double sp2 = Math.min(sp, 1.5);
          h.v = h.v.mul(sp2 / sp);
        }
      }
      h.p = h.p.add(h.v.mul(dt));

      if (h.p.x < -arena) { h.p.x = -arena; h.v.x = Math.abs(h.v.x); }
      if (h.p.x >  arena) { h.p.x =  arena; h.v.x = -Math.abs(h.v.x); }
      if (h.p.y < -arena) { h.p.y = -arena; h.v.y = Math.abs(h.v.y); }
      if (h.p.y >  arena) { h.p.y =  arena; h.v.y = -Math.abs(h.v.y); }
    }
  }

  static double[] linspace(double a, double b, int n) {
    double[] xs = new double[n];
    if (n == 1) { xs[0] = 0.5 * (a + b); return xs; }
    for (int i = 0; i < n; i++) {
      double t = (double)i / (double)(n - 1);
      xs[i] = a + (b - a) * t;
    }
    return xs;
  }

  static double[] planControl(Robot robot, Vec2 goal, List<DiscAgent> humans, Params P) {
    double dt = P.dt;
    int N = (int)Math.max(1.0, Math.round(P.horizon / dt));

    double vLo = clamp(robot.v - P.aV * dt, 0.0, P.vMax);
    double vHi = clamp(robot.v + P.aV * dt, 0.0, P.vMax);
    double wLo = clamp(robot.w - P.aW * dt, -P.wMax, P.wMax);
    double wHi = clamp(robot.w + P.aW * dt, -P.wMax, P.wMax);

    Vec2 toGoal = goal.sub(robot.p);
    double dGoal = norm(toGoal);
    double vPref = (dGoal > 1.0) ? P.vMax : P.vMax * dGoal;

    double[] vSamples = linspace(vLo, vHi, 9);
    double[] wSamples = linspace(wLo, wHi, 11);

    double bestV = 0.0, bestW = 0.0;
    double bestCost = Double.POSITIVE_INFINITY;

    for (double v : vSamples) {
      for (double w : wSamples) {

        Vec2 p = new Vec2(robot.p.x, robot.p.y);
        double th = robot.th;

        double ttcMin = Double.POSITIVE_INFINITY;
        double clearMin = Double.POSITIVE_INFINITY;
        double socSum = 0.0;

        for (int k = 0; k < N; k++) {
          // propagate
          p = new Vec2(p.x + v * Math.cos(th) * dt, p.y + v * Math.sin(th) * dt);
          th = wrapAngle(th + w * dt);
          double t = (k + 1) * dt;

          // predicted humans at time t
          List<DiscAgent> humansT = new ArrayList<>();
          for (DiscAgent h : humans) {
            humansT.add(new DiscAgent(new Vec2(h.p.x + h.v.x * t, h.p.y + h.v.y * t), new Vec2(h.v.x, h.v.y), h.r));
          }

          for (DiscAgent h : humansT) {
            Vec2 rel = new Vec2(p.x - h.p.x, p.y - h.p.y);
            double dist = norm(rel) - (robot.r + h.r);
            if (dist < clearMin) clearMin = dist;

            Vec2 vR = new Vec2(v * Math.cos(th), v * Math.sin(th));
            Vec2 vRel = new Vec2(vR.x - h.v.x, vR.y - h.v.y);
            double ttc = ttcDiscs(rel, vRel, robot.r + h.r);
            if (ttc < ttcMin) ttcMin = ttc;
          }
          socSum += socialCost(p, humansT, P);
        }

        double dTerm = norm(goal.sub(p));
        double eps = 1e-6;

        double cGoal = P.wGoal * dTerm;
        double cClear = P.wClear * (1.0 / (clearMin + eps));
        double cTtc = P.wTtc * (1.0 / (ttcMin + eps));
        double cSoc = P.wSoc * (socSum / (double)N);
        double cVel = P.wVel * ((vPref - v) * (vPref - v));
        double cTurn = P.wTurn * (w * w);
        double cSmooth = P.wSmooth * ((v - robot.v) * (v - robot.v) + 0.1 * (w - robot.w) * (w - robot.w));

        if (clearMin <= 0.0) continue;
        double cost = cGoal + cClear + cTtc + cSoc + cVel + cTurn + cSmooth;

        if (cost < bestCost) {
          bestCost = cost;
          bestV = v;
          bestW = w;
        }
      }
    }
    return new double[]{bestV, bestW};
  }

  public static void main(String[] args) throws IOException {
    Params P = new Params();
    List<DiscAgent> humans = makeCrossingCrowd(12, 0.9, 4L);

    Robot robot = new Robot(new Vec2(-8.0, -8.0), Math.PI / 4.0, 0.0, 0.0, 0.32);
    Vec2 goal = new Vec2(8.0, 8.0);

    double tMax = 70.0;
    int steps = (int)(tMax / P.dt);

    FileWriter csv = new FileWriter("robot_traj_java.csv");
    csv.write("t,x,y,th,v,w\n");

    boolean collision = false;
    boolean reached = false;

    for (int step = 0; step < steps; step++) {
      double t = step * P.dt;

      double[] cmd = planControl(robot, goal, humans, P);
      robot.v = cmd[0];
      robot.w = cmd[1];

      simulateUnicycle(robot, robot.v, robot.w, P.dt);
      updateHumans(humans, P.dt, 9.0, 0.01, 4L + step);

      csv.write(t + "," + robot.p.x + "," + robot.p.y + "," + robot.th + "," + robot.v + "," + robot.w + "\n");

      if (norm(goal.sub(robot.p)) < 0.5) { reached = true; break; }

      for (DiscAgent h : humans) {
        if (norm(robot.p.sub(h.p)) <= (robot.r + h.r)) { collision = true; break; }
      }
      if (collision) break;
    }

    csv.close();

    if (collision) {
      System.out.println("Result: collision. See robot_traj_java.csv");
      System.exit(1);
    } else if (reached) {
      System.out.println("Result: reached goal. See robot_traj_java.csv");
      System.exit(0);
    } else {
      System.out.println("Result: timeout. See robot_traj_java.csv");
      System.exit(0);
    }
  }
}
