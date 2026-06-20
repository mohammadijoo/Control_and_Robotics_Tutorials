// Chapter18_Lesson2.java
// Traversability + terrain classification (toy implementation, no external libs).
//
// Compile: javac Chapter18_Lesson2.java
// Run:     java Chapter18_Lesson2 --points_csv points.csv --resolution 0.25 --out_csv traversability_map.csv
//
// CSV input columns: x,y,z (no header required).
// Output CSV columns: x,y,p_trav,slope_deg,rough_m,step_m,count
//
// Note: In production, Java teams often integrate with ROS2 via rcljava and
// represent maps as nav_msgs/OccupancyGrid or custom layers.

import java.io.*;
import java.util.*;

public class Chapter18_Lesson2 {

  static class GridSpec {
    double xMin, xMax, yMin, yMax, res;
    int width()  { return (int)Math.ceil((xMax - xMin) / res); }
    int height() { return (int)Math.ceil((yMax - yMin) / res); }
  }

  static double sigmoid(double s) { return 1.0 / (1.0 + Math.exp(-s)); }

  static class Points {
    double[] x, y, z;
    Points(double[] x, double[] y, double[] z) { this.x=x; this.y=y; this.z=z; }
    int n() { return x.length; }
  }

  static Points readXYZ(String path) throws IOException {
    ArrayList<Double> xs = new ArrayList<>();
    ArrayList<Double> ys = new ArrayList<>();
    ArrayList<Double> zs = new ArrayList<>();
    try (BufferedReader br = new BufferedReader(new FileReader(path))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (line.trim().isEmpty()) continue;
        String[] parts = line.split(",");
        if (parts.length < 3) continue;
        try {
          xs.add(Double.parseDouble(parts[0].trim()));
          ys.add(Double.parseDouble(parts[1].trim()));
          zs.add(Double.parseDouble(parts[2].trim()));
        } catch (NumberFormatException ex) {
          // allow header
        }
      }
    }
    if (xs.isEmpty()) throw new IllegalArgumentException("No valid points in " + path);
    double[] x = new double[xs.size()], y = new double[ys.size()], z = new double[zs.size()];
    for (int i=0;i<xs.size();i++) { x[i]=xs.get(i); y[i]=ys.get(i); z[i]=zs.get(i); }
    return new Points(x,y,z);
  }

  static void pointsToHeightMap(Points p, GridSpec g, double[] H, int[] C) {
    int Nx = g.width(), Ny = g.height();
    Arrays.fill(H, 0.0);
    Arrays.fill(C, 0);

    for (int i=0;i<p.n();i++) {
      int ix = (int)Math.floor((p.x[i] - g.xMin) / g.res);
      int iy = (int)Math.floor((p.y[i] - g.yMin) / g.res);
      if (ix < 0 || ix >= Nx || iy < 0 || iy >= Ny) continue;
      int k = iy * Nx + ix;
      H[k] += p.z[i];
      C[k] += 1;
    }
    for (int k=0;k<H.length;k++) {
      if (C[k] > 0) H[k] /= (double)C[k];
      else H[k] = Double.NaN;
    }
  }

  static void fillNaNs(double[] H, int Nx, int Ny, int iters) {
    for (int it=0; it<iters; it++) {
      boolean any = false;
      double[] out = Arrays.copyOf(H, H.length);
      for (int y=0;y<Ny;y++) {
        for (int x=0;x<Nx;x++) {
          int k = y*Nx + x;
          if (!Double.isNaN(H[k])) continue;
          any = true;
          double s = 0.0;
          int cnt = 0;
          for (int yy=Math.max(0,y-1); yy<=Math.min(Ny-1,y+1); yy++) {
            for (int xx=Math.max(0,x-1); xx<=Math.min(Nx-1,x+1); xx++) {
              int kk = yy*Nx + xx;
              if (Double.isNaN(H[kk])) continue;
              s += H[kk];
              cnt++;
            }
          }
          if (cnt > 0) out[k] = s / (double)cnt;
        }
      }
      H = System.arraycopy(out, 0, H, 0, H.length) == null ? H : H; // keep H updated
      if (!any) break;
    }
  }

  static void computeFeatures(
      double[] HIn, int[] C, GridSpec g,
      double[] slopeRad, double[] rough, double[] step
  ) {
    int Nx = g.width(), Ny = g.height();
    double[] H = Arrays.copyOf(HIn, HIn.length);
    // fill holes
    for (int it=0; it<6; it++) {
      boolean any = false;
      double[] out = Arrays.copyOf(H, H.length);
      for (int y=0;y<Ny;y++) for (int x=0;x<Nx;x++) {
        int k = y*Nx + x;
        if (!Double.isNaN(H[k])) continue;
        any = true;
        double s = 0.0;
        int cnt = 0;
        for (int yy=Math.max(0,y-1); yy<=Math.min(Ny-1,y+1); yy++) {
          for (int xx=Math.max(0,x-1); xx<=Math.min(Nx-1,x+1); xx++) {
            int kk = yy*Nx + xx;
            if (Double.isNaN(H[kk])) continue;
            s += H[kk];
            cnt++;
          }
        }
        if (cnt > 0) out[k] = s / (double)cnt;
      }
      H = out;
      if (!any) break;
    }

    // slope via central differences
    for (int y=0;y<Ny;y++) for (int x=0;x<Nx;x++) {
      int xm1 = Math.max(0, x-1), xp1 = Math.min(Nx-1, x+1);
      int ym1 = Math.max(0, y-1), yp1 = Math.min(Ny-1, y+1);
      double hx1 = H[y*Nx + xp1], hx0 = H[y*Nx + xm1];
      double hy1 = H[yp1*Nx + x], hy0 = H[ym1*Nx + x];
      double dhdx = (hx1 - hx0) / (2.0 * g.res);
      double dhdy = (hy1 - hy0) / (2.0 * g.res);
      double gn = Math.sqrt(dhdx*dhdx + dhdy*dhdy);
      slopeRad[y*Nx + x] = Math.atan(gn);
    }

    // roughness/std and step/max-min in 3x3 neighborhood
    for (int y=0;y<Ny;y++) for (int x=0;x<Nx;x++) {
      double s = 0.0, s2 = 0.0;
      double mn = Double.POSITIVE_INFINITY, mx = Double.NEGATIVE_INFINITY;
      int cnt = 0;
      for (int yy=Math.max(0,y-1); yy<=Math.min(Ny-1,y+1); yy++) {
        for (int xx=Math.max(0,x-1); xx<=Math.min(Nx-1,x+1); xx++) {
          double v = H[yy*Nx + xx];
          s += v;
          s2 += v*v;
          if (v < mn) mn = v;
          if (v > mx) mx = v;
          cnt++;
        }
      }
      double mu = s / (double)cnt;
      double var = Math.max(0.0, s2 / (double)cnt - mu*mu);
      rough[y*Nx + x] = Math.sqrt(var);
      step[y*Nx + x] = mx - mn;
    }

    // invalidate empty cells
    for (int k=0;k<C.length;k++) {
      if (C[k] == 0) {
        slopeRad[k] = Double.NaN;
        rough[k] = Double.NaN;
        step[k] = Double.NaN;
      }
    }
  }

  static void writeCSV(String outPath, GridSpec g, int[] C, double[] slope, double[] rough, double[] step) throws IOException {
    // illustrative weights (calibrate for your robot)
    double b = 3.0;
    double w1 = -6.0, w2 = -40.0, w3 = -25.0;

    int Nx = g.width(), Ny = g.height();
    try (PrintWriter pw = new PrintWriter(new FileWriter(outPath))) {
      pw.println("x,y,p_trav,slope_deg,rough_m,step_m,count");
      for (int iy=0; iy<Ny; iy++) {
        double yc = g.yMin + (iy + 0.5) * g.res;
        for (int ix=0; ix<Nx; ix++) {
          int k = iy*Nx + ix;
          if (C[k] == 0) continue;
          double s = b + w1*slope[k] + w2*rough[k] + w3*step[k];
          double p = sigmoid(s);
          double slopeDeg = slope[k] * 180.0 / Math.PI;
          double xc = g.xMin + (ix + 0.5) * g.res;
          pw.printf(Locale.US, "%.6f,%.6f,%.8f,%.4f,%.6f,%.6f,%d%n",
                    xc, yc, p, slopeDeg, rough[k], step[k], C[k]);
        }
      }
    }
  }

  public static void main(String[] args) throws Exception {
    String pointsCSV = "";
    String outCSV = "traversability_map.csv";
    double res = 0.25;

    for (int i=0;i<args.length;i++) {
      if (args[i].equals("--points_csv") && i+1<args.length) pointsCSV = args[++i];
      else if (args[i].equals("--out_csv") && i+1<args.length) outCSV = args[++i];
      else if (args[i].equals("--resolution") && i+1<args.length) res = Double.parseDouble(args[++i]);
      else if (args[i].equals("--help")) {
        System.out.println("java Chapter18_Lesson2 --points_csv points.csv --resolution 0.25 --out_csv traversability_map.csv");
        return;
      }
    }

    if (pointsCSV.isEmpty()) throw new IllegalArgumentException("--points_csv required for this Java demo.");

    Points p = readXYZ(pointsCSV);
    double xMin = Arrays.stream(p.x).min().getAsDouble();
    double xMax = Arrays.stream(p.x).max().getAsDouble();
    double yMin = Arrays.stream(p.y).min().getAsDouble();
    double yMax = Arrays.stream(p.y).max().getAsDouble();

    GridSpec g = new GridSpec();
    g.xMin = xMin; g.xMax = xMax; g.yMin = yMin; g.yMax = yMax; g.res = res;

    int Nx = g.width(), Ny = g.height();
    double[] H = new double[Nx*Ny];
    int[] C = new int[Nx*Ny];
    pointsToHeightMap(p, g, H, C);

    double[] slope = new double[Nx*Ny];
    double[] rough = new double[Nx*Ny];
    double[] step = new double[Nx*Ny];
    computeFeatures(H, C, g, slope, rough, step);

    writeCSV(outCSV, g, C, slope, rough, step);
    System.out.println("Exported: " + outCSV);
  }
}
