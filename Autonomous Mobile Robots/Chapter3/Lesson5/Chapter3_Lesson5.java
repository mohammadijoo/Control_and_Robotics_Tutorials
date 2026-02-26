// Chapter3_Lesson5.java
/*
Autonomous Mobile Robots (Control Engineering major)
Chapter 3 — Nonholonomic Motion and Feasibility for AMR
Lesson 5 — Feasibility Checks for Candidate Paths

Java feasibility checker for a car-like robot. Self-contained (no external libs).
Checks:
  - curvature/steering bounds
  - steering-rate bound (via curvature derivative and speed profile)
  - speed & longitudinal acceleration bounds (forward-backward pass)
  - collision clearance on occupancy grid (disk approximation) using Dijkstra

Robotics ecosystem references (not required here):
  - ROSJava (path message integration)
  - JOML (vector math), EJML (linear algebra), JavaCPP OpenCV (distance transform)

Compile/run:
  javac Chapter3_Lesson5.java
  java Chapter3_Lesson5

Author: course content generator
*/

import java.util.*;

public class Chapter3_Lesson5 {

  static class Limits {
    double wheelbase_m = 0.33;
    double delta_max_rad = 0.45;
    double delta_dot_max_rad_s = 0.75;
    double v_max_m_s = 1.2;
    double a_long_max_m_s2 = 0.8;
    double a_lat_max_m_s2 = 1.5;
    double robot_radius_m = 0.25;
    double clearance_margin_m = 0.05;
  }

  static class Point2 {
    double x, y;
    Point2(double x, double y) { this.x = x; this.y = y; }
  }

  static class Report {
    boolean ok;
    String reason;
    double pathLength;
    double kappaAbsMax;
    double kappaMax;
    double vMin;
    double vMax;
    double deltaDotAbsMax;
    double totalTime;
  }

  static double norm(Point2 a) { return Math.hypot(a.x, a.y); }
  static Point2 sub(Point2 a, Point2 b) { return new Point2(a.x - b.x, a.y - b.y); }
  static double cross(Point2 a, Point2 b) { return a.x * b.y - a.y * b.x; }

  static double[] arcLength(List<Point2> xy) {
    int n = xy.size();
    double[] s = new double[n];
    for (int i = 1; i < n; i++) {
      Point2 d = sub(xy.get(i), xy.get(i - 1));
      s[i] = s[i - 1] + norm(d);
    }
    return s;
  }

  static double[] curvatureDiscrete(List<Point2> xy) {
    int n = xy.size();
    double[] kappa = new double[n];
    double eps = 1e-12;
    for (int i = 1; i < n - 1; i++) {
      Point2 p0 = xy.get(i - 1), p1 = xy.get(i), p2 = xy.get(i + 1);
      Point2 a = sub(p1, p0);
      Point2 b = sub(p2, p1);
      Point2 c = sub(p2, p0);
      double la = norm(a), lb = norm(b), lc = norm(c);
      double area2 = cross(a, c);
      double denom = la * lb * lc + eps;
      kappa[i] = 2.0 * area2 / denom;
    }
    if (n >= 3) {
      kappa[0] = kappa[1];
      kappa[n - 1] = kappa[n - 2];
    }
    return kappa;
  }

  static double[] dkappaDs(double[] kappa, double[] s) {
    int n = kappa.length;
    double[] dk = new double[n];
    double eps = 1e-12;
    if (n < 3) return dk;
    for (int i = 1; i < n - 1; i++) {
      double denom = (s[i + 1] - s[i - 1]) + eps;
      dk[i] = (kappa[i + 1] - kappa[i - 1]) / denom;
    }
    dk[0] = (kappa[1] - kappa[0]) / ((s[1] - s[0]) + eps);
    dk[n - 1] = (kappa[n - 1] - kappa[n - 2]) / ((s[n - 1] - s[n - 2]) + eps);
    return dk;
  }

  static void speedProfile(double[] s, double[] kappa, Limits lim, double[] v, double[] dt) {
    int n = s.length;
    double[] vcap = new double[n];
    for (int i = 0; i < n; i++) vcap[i] = lim.v_max_m_s;

    for (int i = 0; i < n; i++) {
      double ak = Math.abs(kappa[i]);
      if (ak > 1e-9) {
        double vlat = Math.sqrt(Math.max(lim.a_lat_max_m_s2 / ak, 0.0));
        vcap[i] = Math.min(vcap[i], vlat);
      }
    }

    double[] w = new double[n];
    for (int i = 0; i < n; i++) {
      double vv = Math.min(vcap[i], lim.v_max_m_s);
      w[i] = vv * vv;
    }

    // forward
    for (int i = 0; i < n - 1; i++) {
      double ds = s[i + 1] - s[i];
      double wNext = w[i] + 2.0 * lim.a_long_max_m_s2 * Math.max(ds, 0.0);
      if (w[i + 1] > wNext) w[i + 1] = wNext;
    }
    // backward
    for (int i = n - 1; i >= 1; i--) {
      double ds = s[i] - s[i - 1];
      double wPrev = w[i] + 2.0 * lim.a_long_max_m_s2 * Math.max(ds, 0.0);
      if (w[i - 1] > wPrev) w[i - 1] = wPrev;
    }

    for (int i = 0; i < n; i++) v[i] = Math.sqrt(Math.max(w[i], 0.0));

    double eps = 1e-12;
    for (int i = 0; i < n - 1; i++) {
      double ds = s[i + 1] - s[i];
      double vAvg = 0.5 * (v[i] + v[i + 1]);
      dt[i] = ds / (vAvg + eps);
    }
  }

  // Multi-source Dijkstra distance-to-obstacle on 8-connected grid.
  static double[] distanceToObstacles(byte[] occ, int H, int W, double resM) {
    double INF = 1e18;
    double[] dist = new double[H * W];
    Arrays.fill(dist, INF);

    class Node {
      double d; int idx;
      Node(double d, int idx) { this.d = d; this.idx = idx; }
    }
    PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a.d));

    for (int i = 0; i < H * W; i++) {
      if (occ[i] == 1) {
        dist[i] = 0.0;
        pq.add(new Node(0.0, i));
      }
    }

    int[] dx = {1, -1, 0, 0,  1,  1, -1, -1};
    int[] dy = {0, 0, 1, -1, 1, -1,  1, -1};
    double[] w = {1, 1, 1, 1, Math.sqrt(2.0), Math.sqrt(2.0), Math.sqrt(2.0), Math.sqrt(2.0)};

    while (!pq.isEmpty()) {
      Node cur = pq.poll();
      if (cur.d > dist[cur.idx]) continue;
      int y = cur.idx / W;
      int x = cur.idx % W;
      for (int k = 0; k < 8; k++) {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
        int nidx = ny * W + nx;
        double nd = cur.d + w[k] * resM;
        if (nd < dist[nidx]) {
          dist[nidx] = nd;
          pq.add(new Node(nd, nidx));
        }
      }
    }
    return dist;
  }

  static Report checkFeasible(
      List<Point2> xy,
      byte[] occ, int H, int W,
      double resM, Point2 origin,
      Limits lim) {

    Report rep = new Report();
    if (xy.size() < 3) {
      rep.ok = false;
      rep.reason = "invalid_path";
      return rep;
    }

    double[] s = arcLength(xy);
    double[] kappa = curvatureDiscrete(xy);
    double[] dkds = dkappaDs(kappa, s);

    double kappaMax = Math.tan(lim.delta_max_rad) / lim.wheelbase_m;
    double kappaAbsMax = 0.0;
    for (double ki : kappa) kappaAbsMax = Math.max(kappaAbsMax, Math.abs(ki));

    rep.kappaMax = kappaMax;
    rep.kappaAbsMax = kappaAbsMax;
    rep.pathLength = s[s.length - 1];

    if (kappaAbsMax > kappaMax + 1e-9) {
      rep.ok = false;
      rep.reason = "curvature_limit_violation";
      return rep;
    }

    double[] v = new double[s.length];
    double[] dt = new double[s.length - 1];
    speedProfile(s, kappa, lim, v, dt);

    rep.vMin = Arrays.stream(v).min().orElse(0.0);
    rep.vMax = Arrays.stream(v).max().orElse(0.0);
    rep.totalTime = Arrays.stream(dt).sum();

    double deltaDotAbsMax = 0.0;
    for (int i = 0; i < s.length; i++) {
      double delta = Math.atan(lim.wheelbase_m * kappa[i]);
      double kappaDot = v[i] * dkds[i];
      double deltaDot = lim.wheelbase_m * Math.pow(Math.cos(delta), 2.0) * kappaDot;
      deltaDotAbsMax = Math.max(deltaDotAbsMax, Math.abs(deltaDot));
    }
    rep.deltaDotAbsMax = deltaDotAbsMax;

    if (deltaDotAbsMax > lim.delta_dot_max_rad_s + 1e-9) {
      rep.ok = false;
      rep.reason = "steering_rate_violation";
      return rep;
    }

    if (occ != null) {
      double[] dist = distanceToObstacles(occ, H, W, resM);
      double needed = lim.robot_radius_m + lim.clearance_margin_m;
      for (int i = 0; i < xy.size(); i++) {
        Point2 p = xy.get(i);
        int gx = (int)Math.floor((p.x - origin.x) / resM);
        int gy = (int)Math.floor((p.y - origin.y) / resM);
        if (gx < 0 || gx >= W || gy < 0 || gy >= H) {
          rep.ok = false;
          rep.reason = "path_outside_grid";
          return rep;
        }
        double c = dist[gy * W + gx];
        if (c < needed) {
          rep.ok = false;
          rep.reason = "collision_violation";
          return rep;
        }
      }
    }

    rep.ok = true;
    rep.reason = "ok";
    return rep;
  }

  public static void main(String[] args) {
    // Demo: arc path
    ArrayList<Point2> path = new ArrayList<>();
    int N = 200;
    double R = 4.0;
    for (int i = 0; i < N; i++) {
      double t = (double)i / (N - 1);
      double ang = 0.6 * t;
      path.add(new Point2(R * Math.sin(ang), R * (1.0 - Math.cos(ang))));
    }

    // Demo occupancy grid 10m x 10m at 0.05m resolution
    double res = 0.05;
    int W = (int)(10.0 / res);
    int H = (int)(10.0 / res);
    byte[] occ = new byte[H * W]; // 0 free, 1 obstacle

    int ox = (int)(2.0 / res);
    int oy = (int)(1.0 / res);
    for (int y = Math.max(0, oy - 3); y < Math.min(H, oy + 3); y++)
      for (int x = Math.max(0, ox - 3); x < Math.min(W, ox + 3); x++)
        occ[y * W + x] = 1;

    Limits lim = new Limits();
    Point2 origin = new Point2(0.0, 0.0);

    Report rep = checkFeasible(path, occ, H, W, res, origin, lim);
    System.out.println("ok=" + rep.ok + " reason=" + rep.reason);
    System.out.println("L=" + rep.pathLength + " kappaAbsMax=" + rep.kappaAbsMax + " kappaMax=" + rep.kappaMax);
    System.out.println("vMin=" + rep.vMin + " vMax=" + rep.vMax + " deltaDotAbsMax=" + rep.deltaDotAbsMax + " T=" + rep.totalTime);
  }
}
