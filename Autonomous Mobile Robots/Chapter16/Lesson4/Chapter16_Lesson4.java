// Chapter16_Lesson4.java
// Prediction-Aware Local Navigation (sampling MPC with chance-constraint penalty)
// Minimal self-contained Java. Compile: javac Chapter16_Lesson4.java ; Run: java Chapter16_Lesson4

import java.util.ArrayList;
import java.util.List;

public class Chapter16_Lesson4 {

  static double wrapAngle(double a) {
    final double pi = Math.PI;
    a = (a + pi) % (2.0 * pi);
    if (a < 0.0) a += 2.0 * pi;
    return a - pi;
  }

  static class Vec2 {
    double x, y;
    Vec2(double x, double y) { this.x = x; this.y = y; }
  }

  static class Vec3 {
    double x, y, th;
    Vec3(double x, double y, double th) { this.x = x; this.y = y; this.th = th; }
  }

  static class Mat2 {
    // row-major
    double a11, a12, a21, a22;
    Mat2(double a11, double a12, double a21, double a22) {
      this.a11 = a11; this.a12 = a12; this.a21 = a21; this.a22 = a22;
    }
  }

  static class Track {
    double px, py, vx, vy;
    Mat2 SigmaPos;
    double radius;
    String name;

    Track(double px, double py, double vx, double vy, Mat2 SigmaPos, double radius, String name) {
      this.px = px; this.py = py; this.vx = vx; this.vy = vy;
      this.SigmaPos = SigmaPos;
      this.radius = radius;
      this.name = name;
    }

    void predict(double dt, double qPos, double qVel) {
      px += dt * vx;
      py += dt * vy;
      double add = (dt * qVel) * (dt * qVel) + qPos * qPos;
      SigmaPos.a11 += add;
      SigmaPos.a22 += add;
    }
  }

  static Vec3 unicycleStep(Vec3 x, double v, double w, double dt) {
    double nx = x.x + dt * v * Math.cos(x.th);
    double ny = x.y + dt * v * Math.sin(x.th);
    double nth = wrapAngle(x.th + dt * w);
    return new Vec3(nx, ny, nth);
  }

  static double zValue(double delta) {
    if (Math.abs(delta - 0.1) < 1e-12) return 1.281551565545;
    if (Math.abs(delta - 0.05) < 1e-12) return 1.644853626951;
    if (Math.abs(delta - 0.02) < 1e-12) return 2.053748910631;
    if (Math.abs(delta - 0.01) < 1e-12) return 2.326347874041;
    if (Math.abs(delta - 0.005) < 1e-12) return 2.575829303549;
    if (Math.abs(delta - 0.001) < 1e-12) return 3.090232306168;
    return 2.326347874041; // fallback
  }

  static double chancePenaltyPoint(Vec2 p, Track t, double RSafe, double delta) {
    double mux = p.x - t.px;
    double muy = p.y - t.py;
    double dist = Math.sqrt(mux * mux + muy * muy) + 1e-9;

    double nx = mux / dist;
    double ny = muy / dist;

    double s2 = nx * (t.SigmaPos.a11 * nx + t.SigmaPos.a12 * ny)
              + ny * (t.SigmaPos.a21 * nx + t.SigmaPos.a22 * ny);
    double s = Math.sqrt(Math.max(0.0, s2) + 1e-12);

    double z = zValue(delta);
    double margin = z * s;
    double violation = Math.max(0.0, margin - (dist - RSafe));
    return violation * violation;
  }

  static double rolloutCost(
      Vec3 x0,
      double v,
      double w,
      Vec2 goal,
      List<Track> tracks0,
      double dt,
      int N,
      double delta,
      double robotRadius) {

    double wGoal = 1.0;
    double wCtrl = 0.05;
    double wRisk = 10.0;

    Vec3 x = new Vec3(x0.x, x0.y, x0.th);
    List<Track> tracks = new ArrayList<>();
    for (Track t : tracks0) {
      tracks.add(new Track(t.px, t.py, t.vx, t.vy,
          new Mat2(t.SigmaPos.a11, t.SigmaPos.a12, t.SigmaPos.a21, t.SigmaPos.a22),
          t.radius, t.name));
    }

    double cost = 0.0;
    for (int k = 0; k < N; ++k) {
      for (Track t : tracks) t.predict(dt, 0.05, 0.2);

      x = unicycleStep(x, v, w, dt);

      double ex = x.x - goal.x;
      double ey = x.y - goal.y;
      cost += wGoal * (ex * ex + ey * ey);

      cost += wCtrl * (v * v + w * w);

      Vec2 p = new Vec2(x.x, x.y);
      double risk = 0.0;
      for (Track t : tracks) {
        double RSafe = robotRadius + t.radius;
        risk += chancePenaltyPoint(p, t, RSafe, delta);
      }
      cost += wRisk * risk;
    }
    return cost;
  }

  public static void main(String[] args) {
    Vec3 x0 = new Vec3(0.0, 0.0, 0.0);
    Vec2 goal = new Vec2(6.0, 0.0);

    List<Track> tracks = new ArrayList<>();
    tracks.add(new Track(3.0, 1.0, 0.0, -0.6, new Mat2(0.15*0.15, 0.0, 0.0, 0.15*0.15), 0.35, "p1"));
    tracks.add(new Track(4.0, -1.2, 0.0, 0.7, new Mat2(0.15*0.15, 0.0, 0.0, 0.15*0.15), 0.35, "p2"));

    double dt = 0.1;
    int N = 25;
    double delta = 0.01;
    double robotRadius = 0.25;

    double vmin = 0.0, vmax = 1.2;
    double wmin = -1.8, wmax = 1.8;
    int nV = 13, nW = 31;

    double bestV = 0.0, bestW = 0.0;
    double bestJ = Double.POSITIVE_INFINITY;

    for (int i = 0; i < nV; ++i) {
      double v = vmin + (vmax - vmin) * (i / (double)(nV - 1));
      for (int j = 0; j < nW; ++j) {
        double w = wmin + (wmax - wmin) * (j / (double)(nW - 1));
        double J = rolloutCost(x0, v, w, goal, tracks, dt, N, delta, robotRadius);
        if (J < bestJ) {
          bestJ = J;
          bestV = v;
          bestW = w;
        }
      }
    }

    System.out.println("Best control u* = [v,w] = [" + bestV + ", " + bestW + "], cost = " + bestJ);
  }
}
