/*
Chapter 9 - Mapping Representations for Mobile Robots
Lesson 1 - Occupancy Grid Mapping

A minimal occupancy grid mapper in Java (no external robotics middleware).
- Stores a log-odds grid
- Uses Bresenham traversal per ray
- Generates a toy world and simulated range readings
- Writes an ASCII PGM image "occupancy_java.pgm"

Compile:
  javac Chapter9_Lesson1.java

Run:
  java Chapter9_Lesson1
*/

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Chapter9_Lesson1 {

  static double clamp(double x, double a, double b) {
    return (x < a) ? a : (x > b) ? b : x;
  }

  static double logit(double p) {
    double eps = 1e-12;
    p = clamp(p, eps, 1.0 - eps);
    return Math.log(p / (1.0 - p));
  }

  static double invLogit(double l) {
    return 1.0 / (1.0 + Math.exp(-l));
  }

  static class Pt {
    int x, y;
    Pt(int x, int y) { this.x = x; this.y = y; }
  }

  static List<Pt> bresenham(int x0, int y0, int x1, int y1) {
    List<Pt> pts = new ArrayList<>();
    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int sx = (x1 >= x0) ? 1 : -1;
    int sy = (y1 >= y0) ? 1 : -1;
    int err = dx - dy;

    int x = x0, y = y0;
    while (true) {
      pts.add(new Pt(x, y));
      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 <  dx) { err += dx; y += sy; }
    }
    return pts;
  }

  static class OccupancyGrid {
    int W, H;
    double res, ox, oy;
    double prior, pOcc, pFree;
    double lMin, lMax, lOcc, lFree;
    double[] L; // row-major y*W + x

    OccupancyGrid(int w, int h, double resolution, double originX, double originY,
                  double priorP, double pOcc, double pFree) {
      this.W = w; this.H = h;
      this.res = resolution;
      this.ox = originX; this.oy = originY;
      this.prior = priorP;
      this.pOcc = pOcc;
      this.pFree = pFree;

      this.lMin = -10.0;
      this.lMax = 10.0;
      double l0 = logit(prior);
      this.L = new double[W * H];
      for (int i = 0; i < L.length; i++) L[i] = l0;

      this.lOcc = logit(pOcc) - logit(prior);
      this.lFree = logit(pFree) - logit(prior);
    }

    boolean inBounds(int gx, int gy) {
      return (0 <= gx && gx < W && 0 <= gy && gy < H);
    }

    int idx(int gx, int gy) {
      return gy * W + gx;
    }

    Pt worldToGrid(double x, double y) {
      int gx = (int)Math.floor((x - ox) / res);
      int gy = (int)Math.floor((y - oy) / res);
      return new Pt(gx, gy);
    }

    void updateRay(double x, double y, double theta, double relAngle, double range, double maxRange) {
      range = clamp(range, 0.0, maxRange);
      double a = theta + relAngle;
      double xe = x + range * Math.cos(a);
      double ye = y + range * Math.sin(a);

      Pt p0 = worldToGrid(x, y);
      Pt p1 = worldToGrid(xe, ye);
      if (!inBounds(p0.x, p0.y)) return;

      List<Pt> cells = bresenham(p0.x, p0.y, p1.x, p1.y);
      if (cells.isEmpty()) return;

      boolean hit = (range < maxRange - 1e-6);
      int nFree = hit ? (cells.size() - 1) : cells.size();

      for (int i = 0; i < nFree; i++) {
        Pt c = cells.get(i);
        if (inBounds(c.x, c.y)) {
          int id = idx(c.x, c.y);
          L[id] = clamp(L[id] + lFree, lMin, lMax);
        }
      }

      if (hit) {
        Pt c = cells.get(cells.size() - 1);
        if (inBounds(c.x, c.y)) {
          int id = idx(c.x, c.y);
          L[id] = clamp(L[id] + lOcc, lMin, lMax);
        }
      }
    }

    double prob(int gx, int gy) {
      return invLogit(L[idx(gx, gy)]);
    }
  }

  static byte[] buildToyWorld(int W, int H) {
    byte[] occ = new byte[W * H];

    // Border walls
    for (int x = 0; x < W; x++) {
      occ[0 * W + x] = 1;
      occ[(H - 1) * W + x] = 1;
    }
    for (int y = 0; y < H; y++) {
      occ[y * W + 0] = 1;
      occ[y * W + (W - 1)] = 1;
    }

    // Rectangle obstacle
    for (int y = 30; y < 50; y++)
      for (int x = 55; x < 75; x++)
        occ[y * W + x] = 1;

    // Small obstacle
    for (int y = 70; y < 80; y++)
      for (int x = 25; x < 35; x++)
        occ[y * W + x] = 1;

    return occ;
  }

  static double castRay(byte[] world, OccupancyGrid grid,
                        double x, double y, double theta, double relAngle,
                        double maxRange, double step) {
    double a = theta + relAngle;
    double dist = 0.0;
    while (dist < maxRange) {
      double xt = x + dist * Math.cos(a);
      double yt = y + dist * Math.sin(a);
      Pt g = grid.worldToGrid(xt, yt);
      if (grid.inBounds(g.x, g.y)) {
        if (world[g.y * grid.W + g.x] == 1) return dist;
      } else {
        return dist;
      }
      dist += step;
    }
    return maxRange;
  }

  static void writePGM(String path, OccupancyGrid grid) throws IOException {
    try (BufferedWriter out = new BufferedWriter(new FileWriter(path))) {
      out.write("P2\n");
      out.write(grid.W + " " + grid.H + "\n");
      out.write("255\n");
      for (int y = grid.H - 1; y >= 0; y--) {
        for (int x = 0; x < grid.W; x++) {
          double p = grid.prob(x, y);
          int pix = (int)Math.round(255.0 * (1.0 - p));
          out.write(Integer.toString(pix));
          out.write(x + 1 < grid.W ? " " : "\n");
        }
      }
    }
  }

  public static void main(String[] args) throws Exception {
    int W = 120, H = 100;
    double res = 0.05;
    OccupancyGrid og = new OccupancyGrid(W, H, res, 0.0, 0.0, 0.5, 0.75, 0.35);
    byte[] world = buildToyWorld(W, H);

    // Trajectory
    double theta = 0.15;
    int steps = 25;

    double fov = Math.PI; // 180 deg
    int nRays = 121;
    double zMax = 3.0;
    double step = res * 0.5;

    for (int k = 0; k < steps; k++) {
      double x = 0.8 + 0.06 * k;
      double y = 0.9 + 0.01 * k;

      for (int i = 0; i < nRays; i++) {
        double ang = -fov / 2.0 + (fov * i) / (nRays - 1);
        double z = castRay(world, og, x, y, theta, ang, zMax, step);
        og.updateRay(x, y, theta, ang, z, zMax);
      }
    }

    writePGM("occupancy_java.pgm", og);
    System.out.println("Wrote occupancy_java.pgm (ASCII PGM).");
  }
}
