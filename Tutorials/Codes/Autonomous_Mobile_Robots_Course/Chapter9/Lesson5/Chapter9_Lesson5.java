// Chapter 9 — Mapping Representations for Mobile Robots
// Lesson 5 (Lab): Build a 2D Occupancy Grid from LiDAR
//
// Self-contained Java program (no external libraries):
// - Simulates robot trajectory and 2D LiDAR in a 2D world with circles + boundary walls
// - Builds occupancy grid using log-odds + Bresenham traversal
// - Writes PGM image (occupancy probability)
//
// Compile:
//   javac Chapter9_Lesson5.java
// Run:
//   java Chapter9_Lesson5

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter9_Lesson5 {

  static class Pose2D {
    double x, y, theta;
    Pose2D(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
  }

  static class Circle {
    double cx, cy, r;
    Circle(double cx, double cy, double r) { this.cx = cx; this.cy = cy; this.r = r; }
  }

  static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  static double logit(double p) {
    p = clamp(p, 1e-9, 1.0 - 1e-9);
    return Math.log(p / (1.0 - p));
  }

  static boolean rayCircleIntersection(double px, double py, double dx, double dy, Circle c, double[] tOut) {
    double ox = px - c.cx;
    double oy = py - c.cy;
    double b = 2.0 * (ox * dx + oy * dy);
    double cterm = ox * ox + oy * oy - c.r * c.r;
    double disc = b * b - 4.0 * cterm;
    if (disc < 0.0) return false;
    double s = Math.sqrt(disc);
    double t1 = (-b - s) / 2.0;
    double t2 = (-b + s) / 2.0;
    double best = 1e300;
    boolean ok = false;
    if (t1 >= 0.0) { best = Math.min(best, t1); ok = true; }
    if (t2 >= 0.0) { best = Math.min(best, t2); ok = true; }
    if (!ok) return false;
    tOut[0] = best;
    return true;
  }

  static boolean rayAABBIntersection(double px, double py, double dx, double dy,
                                     double xmin, double xmax, double ymin, double ymax,
                                     double[] tOut) {
    double tmin = -1e300, tmax = 1e300;

    if (Math.abs(dx) < 1e-12) {
      if (px < xmin || px > xmax) return false;
    } else {
      double tx1 = (xmin - px) / dx;
      double tx2 = (xmax - px) / dx;
      tmin = Math.max(tmin, Math.min(tx1, tx2));
      tmax = Math.min(tmax, Math.max(tx1, tx2));
    }

    if (Math.abs(dy) < 1e-12) {
      if (py < ymin || py > ymax) return false;
    } else {
      double ty1 = (ymin - py) / dy;
      double ty2 = (ymax - py) / dy;
      tmin = Math.max(tmin, Math.min(ty1, ty2));
      tmax = Math.min(tmax, Math.max(ty1, ty2));
    }

    if (tmax < 0.0 || tmin > tmax) return false;
    double t = (tmin >= 0.0) ? tmin : tmax;
    if (t < 0.0) return false;
    tOut[0] = t;
    return true;
  }

  static class OccupancyGrid {
    double res;
    double originX, originY;
    int w, h;
    double l0, lmin, lmax;
    double[] logOdds; // row-major

    OccupancyGrid(double widthM, double heightM, double res, double originX, double originY,
                  double p0, double lmin, double lmax) {
      this.res = res;
      this.originX = originX;
      this.originY = originY;
      this.w = (int)Math.ceil(widthM / res);
      this.h = (int)Math.ceil(heightM / res);
      this.l0 = logit(p0);
      this.lmin = lmin;
      this.lmax = lmax;
      this.logOdds = new double[w * h];
      for (int i = 0; i < w * h; i++) logOdds[i] = this.l0;
    }

    boolean worldToGrid(double x, double y, int[] outIJ) {
      int i = (int)Math.floor((x - originX) / res);
      int j = (int)Math.floor((y - originY) / res);
      if (0 <= i && i < w && 0 <= j && j < h) {
        outIJ[0] = i; outIJ[1] = j;
        return true;
      }
      return false;
    }

    double at(int i, int j) { return logOdds[j * w + i]; }
    void set(int i, int j, double v) { logOdds[j * w + i] = v; }

    static List<int[]> bresenham(int i0, int j0, int i1, int j1) {
      List<int[]> cells = new ArrayList<>();
      int di = Math.abs(i1 - i0);
      int dj = Math.abs(j1 - j0);
      int si = (i0 < i1) ? 1 : -1;
      int sj = (j0 < j1) ? 1 : -1;
      int err = di - dj;
      int i = i0, j = j0;
      while (true) {
        cells.add(new int[]{i, j});
        if (i == i1 && j == j1) break;
        int e2 = 2 * err;
        if (e2 > -dj) { err -= dj; i += si; }
        if (e2 <  di) { err += di; j += sj; }
      }
      return cells;
    }

    void updateRay(Pose2D pose, double angleBody, double r, double zmax,
                   double lOcc, double lFree, double alpha) {
      int[] s = new int[2];
      if (!worldToGrid(pose.x, pose.y, s)) return;
      int i0 = s[0], j0 = s[1];

      double ang = pose.theta + angleBody;
      double ex = pose.x + r * Math.cos(ang);
      double ey = pose.y + r * Math.sin(ang);

      int[] e = new int[2];
      if (!worldToGrid(ex, ey, e)) return;
      int i1 = e[0], j1 = e[1];

      List<int[]> cells = bresenham(i0, j0, i1, j1);
      if (cells.size() <= 1) return;

      boolean hit = (r < (zmax - 0.5 * alpha));
      int last = cells.size() - 1;

      int freeEnd = hit ? (last - 1) : last;
      for (int k = 1; k <= freeEnd; k++) {
        int[] c = cells.get(k);
        int i = c[0], j = c[1];
        double v = clamp(at(i, j) + (lFree - l0), lmin, lmax);
        set(i, j, v);
      }

      if (hit) {
        int[] c = cells.get(last);
        int i = c[0], j = c[1];
        double v = clamp(at(i, j) + (lOcc - l0), lmin, lmax);
        set(i, j, v);
      }
    }

    void writePGM(String path) throws IOException {
      try (FileOutputStream f = new FileOutputStream(path)) {
        String header = "P5\n" + w + " " + h + "\n255\n";
        f.write(header.getBytes(StandardCharsets.US_ASCII));
        for (int j = 0; j < h; j++) {
          for (int i = 0; i < w; i++) {
            double l = at(i, j);
            double p = 1.0 / (1.0 + Math.exp(-l));
            int v = (int)Math.round(clamp(p * 255.0, 0.0, 255.0));
            f.write((byte)(v & 0xFF));
          }
        }
      }
    }
  }

  static double gaussian(Random rng, double mean, double std) {
    // Box-Muller
    double u1 = Math.max(rng.nextDouble(), 1e-12);
    double u2 = rng.nextDouble();
    double z = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
    return mean + std * z;
  }

  public static void main(String[] args) throws Exception {
    // World
    double xmin = -10.0, xmax = 10.0, ymin = -10.0, ymax = 10.0;
    List<Circle> circles = List.of(
        new Circle(-3.0,  2.0, 1.2),
        new Circle( 2.5, -1.0, 1.0),
        new Circle( 4.0,  4.0, 1.5),
        new Circle(-4.5, -4.0, 1.0)
    );

    // LiDAR
    int nBeams = 360;
    double zmax = 8.0;

    // Grid
    double res = 0.1;
    OccupancyGrid grid = new OccupancyGrid(20.0, 20.0, res, -10.0, -10.0, 0.5, -8.0, 8.0);

    // Inverse sensor model params
    double pOcc = 0.70, pFree = 0.30;
    double lOcc = logit(pOcc), lFree = logit(pFree);
    double alpha = 0.2;

    Random rng = new Random(7);

    // Trajectory
    int T = 220;
    List<Pose2D> poses = new ArrayList<>();
    for (int t = 0; t < T; t++) {
      double ang = 2.0 * Math.PI * t / (double)T;
      double x = 6.0 * Math.cos(ang);
      double y = 6.0 * Math.sin(ang);
      double theta = ang + Math.PI / 2.0;
      poses.add(new Pose2D(x, y, theta));
    }

    // Mapping
    double[] tBuf = new double[1];
    for (Pose2D pose : poses) {
      for (int k = 0; k < nBeams; k++) {
        double aBody = -Math.PI + (2.0 * Math.PI) * k / (double)nBeams;
        double ang = pose.theta + aBody;
        double dx = Math.cos(ang);
        double dy = Math.sin(ang);

        double best = 1e300;
        if (rayAABBIntersection(pose.x, pose.y, dx, dy, xmin, xmax, ymin, ymax, tBuf)) {
          best = Math.min(best, tBuf[0]);
        }
        for (Circle c : circles) {
          if (rayCircleIntersection(pose.x, pose.y, dx, dy, c, tBuf)) {
            best = Math.min(best, tBuf[0]);
          }
        }

        double r = zmax;
        if (best < 1e200 && best <= zmax) {
          r = clamp(best + gaussian(rng, 0.0, 0.02), 0.0, zmax);
        }
        grid.updateRay(pose, aBody, r, zmax, lOcc, lFree, alpha);
      }
    }

    grid.writePGM("Chapter9_Lesson5_map_java.pgm");
    System.out.println("Wrote occupancy probability PGM: Chapter9_Lesson5_map_java.pgm");
  }
}
