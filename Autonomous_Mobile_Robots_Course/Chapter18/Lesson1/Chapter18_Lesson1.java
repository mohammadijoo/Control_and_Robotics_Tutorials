// Chapter18_Lesson1.java
// Autonomous Mobile Robots (Control Engineering) — Chapter 18, Lesson 1
// GPS/RTK Integration in Navigation (didactic EKF fusion example)
//
// This Java example implements a compact EKF with state:
//   x = [px, py, yaw, bg]^T
// where bg is gyro bias. Wheel speed is treated as a known input.
//
// GNSS/RTK provides (px, py) in ENU.
//
// Build:
//   javac Chapter18_Lesson1.java
// Run:
//   java Chapter18_Lesson1

import java.util.Random;

public class Chapter18_Lesson1 {

  static double wrapToPi(double a) {
    a = (a + Math.PI) % (2.0 * Math.PI);
    if (a < 0) a += 2.0 * Math.PI;
    a -= Math.PI;
    if (a <= -Math.PI) a += 2.0 * Math.PI;
    return a;
  }

  static class Mat {
    // Small utilities for fixed sizes in this example (4x4, 4x2, 2x2).
    static double[][] eye(int n) {
      double[][] I = new double[n][n];
      for (int i = 0; i < n; i++) I[i][i] = 1.0;
      return I;
    }

    static double[][] add(double[][] A, double[][] B) {
      int r = A.length, c = A[0].length;
      double[][] C = new double[r][c];
      for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
          C[i][j] = A[i][j] + B[i][j];
      return C;
    }

    static double[][] sub(double[][] A, double[][] B) {
      int r = A.length, c = A[0].length;
      double[][] C = new double[r][c];
      for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
          C[i][j] = A[i][j] - B[i][j];
      return C;
    }

    static double[][] mul(double[][] A, double[][] B) {
      int r = A.length, k = A[0].length, c = B[0].length;
      double[][] C = new double[r][c];
      for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
          double s = 0.0;
          for (int t = 0; t < k; t++) s += A[i][t] * B[t][j];
          C[i][j] = s;
        }
      }
      return C;
    }

    static double[][] transpose(double[][] A) {
      int r = A.length, c = A[0].length;
      double[][] T = new double[c][r];
      for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
          T[j][i] = A[i][j];
      return T;
    }

    static double[][] inv2(double[][] A) {
      // Inverse of 2x2
      double a = A[0][0], b = A[0][1], c = A[1][0], d = A[1][1];
      double det = a*d - b*c;
      double[][] X = new double[2][2];
      X[0][0] =  d / det;
      X[0][1] = -b / det;
      X[1][0] = -c / det;
      X[1][1] =  a / det;
      return X;
    }

    static double dot(double[] a, double[] b) {
      double s = 0.0;
      for (int i = 0; i < a.length; i++) s += a[i]*b[i];
      return s;
    }

    static double[] mv(double[][] A, double[] x) {
      int r = A.length, c = A[0].length;
      double[] y = new double[r];
      for (int i = 0; i < r; i++) {
        double s = 0.0;
        for (int j = 0; j < c; j++) s += A[i][j]*x[j];
        y[i] = s;
      }
      return y;
    }
  }

  static class EkfConfig {
    double q_xy  = 0.05;
    double q_yaw = 0.01;
    double q_bg  = 1e-4;

    double r_gps_fix = 1.5*1.5;
    double r_gps_rtk = 0.02*0.02;

    // chi2(2) 0.99
    double gateChi2Dof2 = 9.2103;
  }

  static class EkfGps {
    // x = [px, py, yaw, bg]
    double[] x = new double[4];
    double[][] P = new double[4][4];
    EkfConfig cfg;

    EkfGps(EkfConfig cfg) {
      this.cfg = cfg;
      // init covariance
      P[0][0] = 10.0;
      P[1][1] = 10.0;
      P[2][2] = Math.pow(20.0*Math.PI/180.0, 2);
      P[3][3] = Math.pow(5.0*Math.PI/180.0, 2);
    }

    void setState(double px, double py, double yaw, double bg) {
      x[0] = px; x[1] = py; x[2] = yaw; x[3] = bg;
    }

    void predict(double v_meas, double omega_meas, double dt) {
      double px = x[0], py = x[1], yaw = x[2], bg = x[3];
      double omega = omega_meas - bg;

      double yaw_p = wrapToPi(yaw + omega * dt);
      double px_p = px + v_meas * Math.cos(yaw_p) * dt;
      double py_p = py + v_meas * Math.sin(yaw_p) * dt;

      // F = df/dx (4x4)
      double[][] F = Mat.eye(4);
      F[0][2] = -v_meas * Math.sin(yaw_p) * dt;
      F[1][2] =  v_meas * Math.cos(yaw_p) * dt;
      F[2][3] = -dt;

      // Q (4x4)
      double[][] Q = new double[4][4];
      Q[0][0] = cfg.q_xy * dt;
      Q[1][1] = cfg.q_xy * dt;
      Q[2][2] = cfg.q_yaw * dt;
      Q[3][3] = cfg.q_bg * dt;

      x[0] = px_p; x[1] = py_p; x[2] = yaw_p; x[3] = bg;
      P = Mat.add(Mat.mul(Mat.mul(F, P), Mat.transpose(F)), Q);
    }

    boolean updateGnss(double px_meas, double py_meas, double r_pos) {
      // z = [px, py]
      double[] z = new double[] {px_meas, py_meas};
      double[] h = new double[] {x[0], x[1]};
      double[] innov = new double[] {z[0]-h[0], z[1]-h[1]};

      // H (2x4)
      double[][] H = new double[2][4];
      H[0][0] = 1.0;
      H[1][1] = 1.0;

      // S = H P H' + R
      double[][] R = new double[][] {{r_pos, 0.0},{0.0, r_pos}};
      double[][] S = Mat.add(Mat.mul(Mat.mul(H, P), Mat.transpose(H)), R);
      double[][] Sinv = Mat.inv2(S);

      // d2 = innov' Sinv innov
      double[] Sinv_innov = Mat.mv(Sinv, innov);
      double d2 = Mat.dot(innov, Sinv_innov);
      if (d2 > cfg.gateChi2Dof2) return false;

      // K = P H' S^-1  => (4x2)
      double[][] K = Mat.mul(Mat.mul(P, Mat.transpose(H)), Sinv);

      // x = x + K innov
      x[0] += K[0][0]*innov[0] + K[0][1]*innov[1];
      x[1] += K[1][0]*innov[0] + K[1][1]*innov[1];
      x[2] = wrapToPi(x[2] + K[2][0]*innov[0] + K[2][1]*innov[1]);
      x[3] += K[3][0]*innov[0] + K[3][1]*innov[1];

      // P = (I - K H) P
      double[][] I = Mat.eye(4);
      double[][] KH = Mat.mul(K, H);
      P = Mat.mul(Mat.sub(I, KH), P);
      return true;
    }
  }

  static double rtkQualityToRpos(EkfConfig cfg, int fixQ) {
    if (fixQ == 4) return cfg.r_gps_rtk;     // RTK fixed
    if (fixQ == 5) return 0.2*0.2;           // float
    if (fixQ == 2) return 0.8*0.8;           // DGPS
    return cfg.r_gps_fix;
  }

  public static void main(String[] args) {
    EkfConfig cfg = new EkfConfig();
    EkfGps ekf = new EkfGps(cfg);
    ekf.setState(0.0, 0.0, 0.0, 0.0);

    double dt = 0.05;
    double T = 60.0;
    int n = (int)(T/dt);

    double vTrue = 1.2;
    double omegaTrue = 0.07;
    double bgTrue = 0.02;

    Random rng = new Random(42);

    double px = 0.0, py = 0.0, yaw = 0.0;

    System.out.println("t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,gnss_ok");
    for (int k = 0; k < n; k++) {
      double t = k*dt;

      yaw = wrapToPi(yaw + omegaTrue * dt);
      px += vTrue * Math.cos(yaw) * dt;
      py += vTrue * Math.sin(yaw) * dt;

      double vMeas = vTrue + rng.nextGaussian()*0.05;
      double omegaMeas = omegaTrue + bgTrue + rng.nextGaussian()*0.01;

      ekf.predict(vMeas, omegaMeas, dt);

      boolean ok = false;
      if (k % (int)(0.2/dt) == 0) {
        int fixQ = (k % 200 != 0) ? 4 : 5;
        double rPos = rtkQualityToRpos(cfg, fixQ);
        double gnssPx = px + rng.nextGaussian()*Math.sqrt(rPos);
        double gnssPy = py + rng.nextGaussian()*Math.sqrt(rPos);
        ok = ekf.updateGnss(gnssPx, gnssPy, rPos);
      }

      System.out.printf("%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d%n",
        t, px, py, yaw, ekf.x[0], ekf.x[1], ekf.x[2], ok ? 1 : 0);
    }
  }
}
