// Chapter17_Lesson3.java
// Autonomous Mobile Robots (Control Engineering) — Chapter 17, Lesson 3
// Next-Best-View (NBV) Strategies — compact Java reference implementation
//
// Compile:
//   javac Chapter17_Lesson3.java
// Run:
//   java Chapter17_Lesson3
//
// This code mirrors the logic of the C++ demo: choose an NBV among random candidates using
// U(v) = EIG(v) - lambda * distance(v,current).

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter17_Lesson3 {

  static double clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  static double bernoulliEntropy(double p) {
    double eps = 1e-12;
    p = clamp(p, eps, 1.0 - eps);
    return -p * Math.log(p) - (1.0 - p) * Math.log(1.0 - p);
  }

  static class SensorBinaryModel {
    double pHit = 0.85;   // P(z=occ | m=occ)
    double pFalse = 0.15; // P(z=occ | m=free)

    double posterior(double pOcc, boolean zOcc) {
      double p = clamp(pOcc, 1e-12, 1.0 - 1e-12);
      double num, den;
      if (zOcc) {
        num = pHit * p;
        den = pHit * p + pFalse * (1.0 - p);
      } else {
        num = (1.0 - pHit) * p;
        den = (1.0 - pHit) * p + (1.0 - pFalse) * (1.0 - p);
      }
      return clamp(num / den, 1e-12, 1.0 - 1e-12);
    }

    double expectedPosteriorEntropy(double pOcc) {
      double p = clamp(pOcc, 1e-12, 1.0 - 1e-12);
      double pZOcc = pHit * p + pFalse * (1.0 - p);
      double pZFree = 1.0 - pZOcc;
      double pPostOcc = posterior(p, true);
      double pPostFree = posterior(p, false);
      return pZOcc * bernoulliEntropy(pPostOcc) + pZFree * bernoulliEntropy(pPostFree);
    }

    double expectedInformationGain(double pOcc) {
      return bernoulliEntropy(pOcc) - expectedPosteriorEntropy(pOcc);
    }
  }

  static class GridBelief {
    int W, H;
    double[] p; // p[y*W + x]

    GridBelief(int w, int h) {
      W = w; H = h;
      p = new double[w*h];
      for (int i = 0; i < p.length; i++) p[i] = 0.5;
    }

    boolean inBounds(int x, int y) {
      return (0 <= x && x < W && 0 <= y && y < H);
    }

    double at(int x, int y) {
      return p[y*W + x];
    }
  }

  static List<int[]> bresenham(int x0, int y0, int x1, int y1) {
    List<int[]> pts = new ArrayList<>();
    int dx = Math.abs(x1 - x0);
    int dy = -Math.abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true) {
      pts.add(new int[]{x, y});
      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
    return pts;
  }

  static int[] rayEndpoint(int cx, int cy, double theta, double r) {
    int ex = (int)Math.round(cx + r * Math.cos(theta));
    int ey = (int)Math.round(cy + r * Math.sin(theta));
    return new int[]{ex, ey};
  }

  static double expectedIgView(
      GridBelief grid,
      int cx, int cy, double heading,
      SensorBinaryModel sensor,
      double fovRad, int nRays, double maxRange,
      double occStop
  ) {
    if (!grid.inBounds(cx, cy)) return -1e9;
    double total = 0.0;
    for (int i = 0; i < nRays; i++) {
      double a = heading - 0.5*fovRad + ((double)i / Math.max(1, nRays-1)) * fovRad;
      int[] end = rayEndpoint(cx, cy, a, maxRange);
      List<int[]> line = bresenham(cx, cy, end[0], end[1]);
      for (int k = 1; k < line.size(); k++) {
        int x = line.get(k)[0], y = line.get(k)[1];
        if (!grid.inBounds(x, y)) break;
        double pOcc = grid.at(x, y);
        total += sensor.expectedInformationGain(pOcc);
        if (pOcc > occStop) break;
      }
    }
    return total;
  }

  static double distCost(int x0, int y0, int x1, int y1) {
    return Math.hypot(x1 - x0, y1 - y0);
  }

  public static void main(String[] args) {
    int W = 30, H = 18;
    GridBelief grid = new GridBelief(W, H);
    SensorBinaryModel sensor = new SensorBinaryModel();
    Random rng = new Random(7);

    int cx = 2, cy = 2;
    double heading = 0.0;

    int N = 120;
    double lambda = 0.22;

    double bestU = -1e18;
    int bestX = cx, bestY = cy;
    double bestTh = heading, bestIg = 0.0;

    for (int i = 0; i < N; i++) {
      int vx = rng.nextInt(W);
      int vy = rng.nextInt(H);
      double th = rng.nextDouble() * 2.0 * Math.PI;

      double ig = expectedIgView(grid, vx, vy, th, sensor, Math.PI, 45, 10.0, 0.70);
      double c = distCost(cx, cy, vx, vy);
      double u = ig - lambda * c;

      if (u > bestU) {
        bestU = u;
        bestX = vx; bestY = vy; bestTh = th; bestIg = ig;
      }
    }

    System.out.println("NBV = (" + bestX + "," + bestY + "), heading=" + bestTh
        + ", IG=" + bestIg + ", U=" + bestU);
    System.out.println("(This demo only selects a view; a real system would plan & move, then update belief.)");
  }
}
