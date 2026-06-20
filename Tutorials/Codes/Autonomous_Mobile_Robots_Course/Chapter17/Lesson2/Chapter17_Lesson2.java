/*
Chapter 17 - Lesson 2: Information Gain and Entropy Reduction
Autonomous Mobile Robots (Control Engineering)

Minimal Java implementation for:
- Bernoulli entropy
- Per-cell information gain for a binary sensor model
- Simple expected IG scoring for candidate viewpoints via ray stepping

Compile:
  javac Chapter17_Lesson2.java
Run:
  java Chapter17_Lesson2
*/

import java.util.HashSet;
import java.util.Random;
import java.util.Set;

public class Chapter17_Lesson2 {

  static double clip(double x, double lo, double hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  static double bernoulliEntropy(double p) {
    double eps = 1e-12;
    p = clip(p, eps, 1.0 - eps);
    return -p * Math.log(p) - (1.0 - p) * Math.log(1.0 - p);
  }

  static double infoGainCell(double p, double pHit, double pFalse) {
    double Hprior = bernoulliEntropy(p);

    double PzOcc = pHit * p + pFalse * (1.0 - p);
    double PzFree = 1.0 - PzOcc;

    double eps = 1e-15;
    PzOcc = clip(PzOcc, eps, 1.0 - eps);
    PzFree = clip(PzFree, eps, 1.0 - eps);

    double pPostOcc = (pHit * p) / PzOcc;
    double pPostFree = ((1.0 - pHit) * p) / PzFree;

    double Hpost = PzOcc * bernoulliEntropy(pPostOcc) + PzFree * bernoulliEntropy(pPostFree);
    return Hprior - Hpost;
  }

  static class Grid {
    final int width, height;
    final double resolution, originX, originY;
    final double[] p; // row-major

    Grid(int w, int h, double res, double ox, double oy) {
      width = w;
      height = h;
      resolution = res;
      originX = ox;
      originY = oy;
      p = new double[w * h];
      for (int i = 0; i < p.length; i++) p[i] = 0.5;
    }

    boolean inBounds(int ix, int iy) {
      return (0 <= ix && ix < width && 0 <= iy && iy < height);
    }

    int idx(int ix, int iy) {
      return iy * width + ix;
    }

    double get(int ix, int iy) {
      return p[idx(ix, iy)];
    }

    void set(int ix, int iy, double v) {
      p[idx(ix, iy)] = v;
    }

    int[] worldToGrid(double x, double y) {
      int ix = (int) Math.floor((x - originX) / resolution);
      int iy = (int) Math.floor((y - originY) / resolution);
      return new int[] {ix, iy};
    }

    double mapEntropy() {
      double s = 0.0;
      for (double prob : p) s += bernoulliEntropy(prob);
      return s;
    }
  }

  static double expectedInfoGainView(
      Grid g,
      double x,
      double y,
      double yaw,
      double fovDeg,
      int nRays,
      double maxRange,
      double pHit,
      double pFalse,
      double unknownLo,
      double unknownHi,
      double occStopThreshold) {

    double fov = Math.toRadians(fovDeg);
    double[] angles = new double[Math.max(nRays, 1)];
    if (nRays < 2) {
      angles[0] = 0.0;
    } else {
      for (int i = 0; i < nRays; i++) {
        angles[i] = (-0.5 * fov) + i * (fov / (nRays - 1));
      }
    }

    Set<Long> visited = new HashSet<>();
    double igTotal = 0.0;
    double step = 0.5 * g.resolution;

    for (double a : angles) {
      double theta = yaw + a;

      for (double t = 0.0; t <= maxRange; t += step) {
        double xr = x + t * Math.cos(theta);
        double yr = y + t * Math.sin(theta);
        int[] ij = g.worldToGrid(xr, yr);
        int ix = ij[0], iy = ij[1];
        if (!g.inBounds(ix, iy)) break;

        long key = (((long) ix) << 32) ^ (iy & 0xffffffffL);
        if (visited.contains(key)) continue;
        visited.add(key);

        double p = g.get(ix, iy);
        if (p >= occStopThreshold) break; // occlusion
        if (unknownLo <= p && p <= unknownHi) {
          igTotal += infoGainCell(p, pHit, pFalse);
        }
      }
    }

    return igTotal;
  }

  public static void main(String[] args) {
    Grid g = new Grid(120, 120, 0.1, -6.0, -6.0);
    Random rng = new Random(0);

    for (int k = 0; k < 4000; k++) {
      int ix = rng.nextInt(g.width);
      int iy = rng.nextInt(g.height);
      double v = clip(0.15 + 0.10 * rng.nextGaussian(), 0.01, 0.99);
      g.set(ix, iy, v);
    }
    for (int k = 0; k < 1200; k++) {
      int ix = rng.nextInt(g.width);
      int iy = rng.nextInt(g.height);
      double v = clip(0.85 + 0.08 * rng.nextGaussian(), 0.01, 0.99);
      g.set(ix, iy, v);
    }

    System.out.println("Initial map entropy (nats): " + g.mapEntropy());

    double[][] poses = new double[][] {
      {0.0, 0.0, 0.0},
      {-2.0, 1.5, 0.7},
      {2.0, -1.0, -1.2}
    };

    for (double[] pose : poses) {
      double ig = expectedInfoGainView(g, pose[0], pose[1], pose[2],
          180.0, 181, 6.0, 0.85, 0.15, 0.4, 0.6, 0.75);
      System.out.printf("Pose=(%.2f, %.2f, %.2f)  Expected IG≈ %.3f nats%n",
          pose[0], pose[1], pose[2], ig);
    }
  }
}
