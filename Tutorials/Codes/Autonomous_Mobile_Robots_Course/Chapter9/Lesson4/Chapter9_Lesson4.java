// Chapter9_Lesson4.java
// Elevation + traversability maps (outdoor AMR) — minimal Java implementation (no external deps).
// Output: CSV files for elevation and traversability.

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class Chapter9_Lesson4 {

  static class ElevationTraversabilityMap {
    final double xMin, xMax, yMin, yMax, res;
    final int nx, ny;
    final double R;
    final double gateK;

    final double[][] mu;
    final double[][] sigma2;
    final int[][] count;

    ElevationTraversabilityMap(double xMin, double xMax, double yMin, double yMax,
                              double res, double sigma0, double measSigma, double gateK) {
      this.xMin = xMin; this.xMax = xMax;
      this.yMin = yMin; this.yMax = yMax;
      this.res = res;
      this.nx = (int) Math.ceil((xMax - xMin) / res);
      this.ny = (int) Math.ceil((yMax - yMin) / res);
      this.R = measSigma * measSigma;
      this.gateK = gateK;

      mu = new double[nx][ny];
      sigma2 = new double[nx][ny];
      count = new int[nx][ny];

      double s2 = sigma0 * sigma0;
      for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
          sigma2[i][j] = s2;
          mu[i][j] = 0.0;
          count[i][j] = 0;
        }
      }
    }

    boolean updatePoint(double x, double y, double z) {
      int ix = (int) ((x - xMin) / res);
      int iy = (int) ((y - yMin) / res);
      if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) return false;

      double m = mu[ix][iy];
      double s2 = sigma2[ix][iy];

      double nu = z - m;
      double S = s2 + R;

      if (count[ix][iy] > 0 && Math.abs(nu) > gateK * Math.sqrt(S)) {
        return false;
      }

      double K = s2 / S;
      mu[ix][iy] = m + K * nu;
      sigma2[ix][iy] = (1.0 - K) * s2;
      count[ix][iy] += 1;
      return true;
    }

    int updatePointCloud(double[][] pts) {
      int acc = 0;
      for (int k = 0; k < pts.length; k++) {
        acc += updatePoint(pts[k][0], pts[k][1], pts[k][2]) ? 1 : 0;
      }
      return acc;
    }

    static double sigmoid(double x) {
      return 1.0 / (1.0 + Math.exp(-x));
    }

    void computeFeatures(double[][] slope, double[][] rough, double[][] step) {
      for (int i = 1; i < nx - 1; i++) {
        for (int j = 1; j < ny - 1; j++) {
          if (count[i][j] == 0) continue;

          double dzdx = (mu[i + 1][j] - mu[i - 1][j]) / (2.0 * res);
          double dzdy = (mu[i][j + 1] - mu[i][j - 1]) / (2.0 * res);
          slope[i][j] = Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy));

          double sum = 0.0;
          double sum2 = 0.0;
          int n = 0;
          double maxdiff = 0.0;

          for (int di = -1; di <= 1; di++) {
            for (int dj = -1; dj <= 1; dj++) {
              double v = mu[i + di][j + dj];
              sum += v;
              sum2 += v * v;
              n += 1;
              maxdiff = Math.max(maxdiff, Math.abs(v - mu[i][j]));
            }
          }
          double mean = sum / n;
          double var = Math.max(0.0, sum2 / n - mean * mean);
          rough[i][j] = Math.sqrt(var);
          step[i][j] = maxdiff;
        }
      }
    }

    double[][] traversabilityCost(double[][] slope, double[][] rough, double[][] step,
                                  double slopeRef, double roughRef, double stepRef,
                                  double wS, double wR, double wD, double bias) {
      double[][] cost = new double[nx][ny];
      double invS = 1.0 / Math.max(1e-9, slopeRef);
      double invR = 1.0 / Math.max(1e-9, roughRef);
      double invD = 1.0 / Math.max(1e-9, stepRef);

      for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
          double s = slope[i][j] * invS;
          double r = rough[i][j] * invR;
          double d = step[i][j] * invD;
          double score = wS * s + wR * r + wD * d + bias;
          cost[i][j] = sigmoid(score);
        }
      }
      return cost;
    }

    void saveCSV(String path, double[][] grid) throws IOException {
      try (BufferedWriter bw = new BufferedWriter(new FileWriter(path))) {
        for (int j = 0; j < ny; j++) {
          for (int i = 0; i < nx; i++) {
            bw.write(Double.toString(grid[i][j]));
            if (i + 1 < nx) bw.write(",");
          }
          bw.newLine();
        }
      }
    }

    void saveCSVInt(String path, int[][] grid) throws IOException {
      try (BufferedWriter bw = new BufferedWriter(new FileWriter(path))) {
        for (int j = 0; j < ny; j++) {
          for (int i = 0; i < nx; i++) {
            bw.write(Integer.toString(grid[i][j]));
            if (i + 1 < nx) bw.write(",");
          }
          bw.newLine();
        }
      }
    }
  }

  static double terrainTrue(double x, double y) {
    double hill = 0.25 * Math.exp(-0.08 * ((x - 2.0) * (x - 2.0) + (y + 1.0) * (y + 1.0)));
    return 0.20 * Math.sin(0.35 * x) + 0.15 * Math.cos(0.25 * y) + 0.03 * x + hill;
  }

  static double[][] generateSyntheticPointCloud(int nGround, int nOutliers, long seed) {
    Random rng = new Random(seed);
    double[][] pts = new double[nGround + nOutliers][3];

    for (int i = 0; i < nGround; i++) {
      double x = -10.0 + 20.0 * rng.nextDouble();
      double y = -10.0 + 20.0 * rng.nextDouble();
      double z = terrainTrue(x, y) + 0.10 * rng.nextGaussian();
      pts[i][0] = x; pts[i][1] = y; pts[i][2] = z;
    }
    for (int i = 0; i < nOutliers; i++) {
      double x = -10.0 + 20.0 * rng.nextDouble();
      double y = -10.0 + 20.0 * rng.nextDouble();
      double z = terrainTrue(x, y) + 0.8 + 0.05 * rng.nextGaussian();
      pts[nGround + i][0] = x; pts[nGround + i][1] = y; pts[nGround + i][2] = z;
    }

    for (int i = pts.length - 1; i > 0; i--) {
      int j = rng.nextInt(i + 1);
      double[] tmp = pts[i];
      pts[i] = pts[j];
      pts[j] = tmp;
    }
    return pts;
  }

  public static void main(String[] args) throws Exception {
    ElevationTraversabilityMap emap = new ElevationTraversabilityMap(
        -10.0, 10.0, -10.0, 10.0, 0.20,
        2.0, 0.10, 3.0
    );

    double[][] pts = generateSyntheticPointCloud(50000, 2000, 7);
    int accepted = emap.updatePointCloud(pts);
    System.out.println("Points accepted: " + accepted + " out of " + pts.length);

    double[][] slope = new double[emap.nx][emap.ny];
    double[][] rough = new double[emap.nx][emap.ny];
    double[][] step  = new double[emap.nx][emap.ny];
    emap.computeFeatures(slope, rough, step);

    double[][] cost = emap.traversabilityCost(
        slope, rough, step,
        0.35, 0.08, 0.12,
        3.0, 2.0, 2.5,
        -3.0
    );

    emap.saveCSV("Chapter9_Lesson4_elevation_mu.csv", emap.mu);
    emap.saveCSV("Chapter9_Lesson4_traversability_cost.csv", cost);
    emap.saveCSVInt("Chapter9_Lesson4_fusion_count.csv", emap.count);

    System.out.println("Saved CSV files.");
  }
}
