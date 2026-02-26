// Chapter9_Lesson2.java
// Autonomous Mobile Robots — Chapter 9, Lesson 2
// Log-Odds Updates and Bayesian Cells (Occupancy Grid Mapping)
//
// Minimal Java example that maintains a log-odds occupancy grid, updates with an inverse
// sensor model along rays (Bresenham), and writes a PGM image.
//
// Compile:
//   javac Chapter9_Lesson2.java
// Run:
//   java Chapter9_Lesson2
//
// Output: occupancy_java.pgm

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public class Chapter9_Lesson2 {

  static float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }

  static float logit(float p) {
    float eps = 1e-6f;
    p = clamp(p, eps, 1.0f - eps);
    return (float) Math.log(p / (1.0f - p));
  }

  static float logistic(float l) {
    if (l >= 0.0f) {
      float z = (float) Math.exp(-l);
      return 1.0f / (1.0f + z);
    } else {
      float z = (float) Math.exp(l);
      return z / (1.0f + z);
    }
  }

  static class Grid {
    final int width, height;
    final float res, originX, originY;
    final float[] logOdds; // row-major

    Grid(int w, int h, float res, float ox, float oy) {
      this.width = w;
      this.height = h;
      this.res = res;
      this.originX = ox;
      this.originY = oy;
      this.logOdds = new float[w * h];
    }

    boolean inBounds(int ix, int iy) {
      return (0 <= ix && ix < width && 0 <= iy && iy < height);
    }

    int idx(int ix, int iy) {
      return iy * width + ix;
    }

    int[] worldToMap(float x, float y) {
      int ix = Math.round((x - originX) / res);
      int iy = Math.round((y - originY) / res);
      return new int[] {ix, iy};
    }

    void updateCell(int ix, int iy, float delta, float lmin, float lmax) {
      if (!inBounds(ix, iy)) return;
      int k = idx(ix, iy);
      logOdds[k] = clamp(logOdds[k] + delta, lmin, lmax);
    }
  }

  static List<int[]> bresenham(int x0, int y0, int x1, int y1) {
    List<int[]> pts = new ArrayList<>();
    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int sx = (x1 >= x0) ? 1 : -1;
    int sy = (y1 >= y0) ? 1 : -1;
    int x = x0, y = y0;

    if (dy <= dx) {
      int err = dx / 2;
      while (x != x1) {
        pts.add(new int[] {x, y});
        err -= dy;
        if (err < 0) {
          y += sy;
          err += dx;
        }
        x += sx;
      }
      pts.add(new int[] {x1, y1});
    } else {
      int err = dy / 2;
      while (y != y1) {
        pts.add(new int[] {x, y});
        err -= dx;
        if (err < 0) {
          x += sx;
          err += dy;
        }
        y += sy;
      }
      pts.add(new int[] {x1, y1});
    }
    return pts;
  }

  static float raycastCircles(float x, float y, float th, float a, float rangeMax, float[][] circles) {
    float dx = (float) Math.cos(th + a);
    float dy = (float) Math.sin(th + a);
    float step = 0.02f;
    for (float t = 0.0f; t <= rangeMax; t += step) {
      float px = x + t * dx;
      float py = y + t * dy;
      for (float[] c : circles) {
        float cx = c[0], cy = c[1], rr = c[2];
        float ex = px - cx, ey = py - cy;
        if (ex * ex + ey * ey <= rr * rr) {
          return Math.max(0.05f, t);
        }
      }
    }
    return rangeMax;
  }

  static void updateScan(Grid g, float x, float y, float th,
                         float[] angles, float[] ranges, float rangeMax,
                         float p0, float pOcc, float pFree,
                         float lmin, float lmax) {
    float l0 = logit(p0);
    float lOcc = logit(pOcc) - l0;
    float lFree = logit(pFree) - l0;

    int[] s = g.worldToMap(x, y);
    int sx = s[0], sy = s[1];

    for (int i = 0; i < angles.length; i++) {
      float a = angles[i];
      float r = ranges[i];
      if (!Float.isFinite(r) || r <= 0.0f) continue;
      float rEff = Math.min(r, rangeMax);

      float bx = x + rEff * (float) Math.cos(th + a);
      float by = y + rEff * (float) Math.sin(th + a);
      int[] e = g.worldToMap(bx, by);
      int ex = e[0], ey = e[1];

      List<int[]> ray = bresenham(sx, sy, ex, ey);
      if (ray.isEmpty()) continue;

      for (int k = 0; k + 1 < ray.size(); k++) {
        int[] p = ray.get(k);
        g.updateCell(p[0], p[1], lFree, lmin, lmax);
      }
      if (r < rangeMax - 1e-6f) {
        int[] p = ray.get(ray.size() - 1);
        g.updateCell(p[0], p[1], lOcc, lmin, lmax);
      }
    }
  }

  static void savePGM(Grid g, String path) throws IOException {
    try (FileOutputStream out = new FileOutputStream(path)) {
      String header = "P5\n" + g.width + " " + g.height + "\n255\n";
      out.write(header.getBytes(StandardCharsets.US_ASCII));

      for (int iy = g.height - 1; iy >= 0; iy--) {
        for (int ix = 0; ix < g.width; ix++) {
          float l = g.logOdds[g.idx(ix, iy)];
          float p = logistic(l);
          int v = Math.round(clamp(255.0f * (1.0f - p), 0.0f, 255.0f));
          out.write((byte) (v & 0xFF));
        }
      }
    }
  }

  public static void main(String[] args) throws IOException {
    Grid grid = new Grid(220, 220, 0.05f, -5.5f, -5.5f);

    float[][] circles = new float[][] {
      {1.5f, 0.5f, 0.6f},
      {-1.2f, 1.2f, 0.5f},
    };

    // Angles
    int N = 121;
    float[] angles = new float[N];
    for (int i = 0; i < N; i++) {
      angles[i] = (float) (-Math.PI / 2.0 + (double) i * (Math.PI / (N - 1)));
    }
    float rangeMax = 6.0f;

    float[][] poses = new float[][] {
      {-2.0f, -2.0f, 0.2f},
      {0.0f, -2.5f, 0.6f},
      {2.0f, -1.5f, 1.0f},
      {2.0f, 1.0f, 1.6f},
      {0.0f, 2.5f, 2.5f},
      {-2.0f, 2.0f, -2.7f},
    };

    for (float[] pose : poses) {
      float x = pose[0], y = pose[1], th = pose[2];
      float[] ranges = new float[N];
      for (int i = 0; i < N; i++) {
        ranges[i] = raycastCircles(x, y, th, angles[i], rangeMax, circles);
      }
      updateScan(grid, x, y, th, angles, ranges, rangeMax,
                 0.5f, 0.7f, 0.3f, -10.0f, 10.0f);
    }

    savePGM(grid, "occupancy_java.pgm");
    System.out.println("Wrote occupancy_java.pgm");
  }
}
