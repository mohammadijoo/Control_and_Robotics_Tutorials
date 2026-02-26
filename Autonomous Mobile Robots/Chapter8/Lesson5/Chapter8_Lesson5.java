// Chapter8_Lesson5.java
// Autonomous Mobile Robots — Chapter 8 (Particle-Filter Localization)
// Lesson 5 Lab: Implement MCL on a Map
//
// Minimal Java 17 MCL on an occupancy grid. Writes CSV trajectories.
//
// Compile:
//   javac Chapter8_Lesson5.java
// Run:
//   java Chapter8_Lesson5

import java.io.*;
import java.util.*;

public class Chapter8_Lesson5 {
  static final double PI = Math.PI;

  static double wrap(double a) {
    a = (a + PI) % (2.0 * PI);
    if (a < 0) a += 2.0 * PI;
    return a - PI;
  }

  static class GridMap {
    int W = 220, H = 160;
    double res = 0.05;
    byte[] occ = new byte[W * H];

    byte at(int x, int y) {
      if (x < 0 || x >= W || y < 0 || y >= H) return 1;
      return occ[y * W + x];
    }
    void worldToGrid(double wx, double wy, int[] g) {
      g[0] = (int)(wx / res);
      g[1] = (int)(wy / res);
    }
    void rect(int x0,int y0,int x1,int y1) {
      for (int y=y0; y<y1; y++)
        for (int x=x0; x<x1; x++)
          occ[y*W + x] = 1;
    }
    void buildDemo() {
      for (int x=0; x<W; x++) { occ[0*W+x]=1; occ[(H-1)*W+x]=1; }
      for (int y=0; y<H; y++) { occ[y*W+0]=1; occ[y*W+(W-1)]=1; }
      rect(40, 30, 160, 55);
      rect(20, 85, 90, 110);
      rect(140, 85, 170, 140);
      rect(180, 15, 205, 40);
    }
  }

  static double rayCast(GridMap m, double x, double y, double th, double rMax, double step) {
    double r = 0.0;
    int[] g = new int[2];
    while (r < rMax) {
      double wx = x + r * Math.cos(th);
      double wy = y + r * Math.sin(th);
      m.worldToGrid(wx, wy, g);
      if (m.at(g[0], g[1]) != 0) return r;
      r += step;
    }
    return rMax;
  }

  static class Particle { double x, y, th, w; }

  static int[] systematicResample(double[] w, Random rng) {
    int N = w.length;
    double r0 = rng.nextDouble() / N;
    double[] cdf = new double[N];
    double s = 0.0;
    for (int i=0;i<N;i++){ s += w[i]; cdf[i]=s; }
    int[] idx = new int[N];
    int j = 0;
    for (int i=0;i<N;i++){
      double u = r0 + (double)i / N;
      while (j < N-1 && u > cdf[j]) j++;
      idx[i]=j;
    }
    return idx;
  }

  static class MCL {
    GridMap m;
    int N = 1200;
    double rMax = 6.0;
    double[] beams = new double[13];
    double sigmaHit = 0.20, zHit=0.85, zRand=0.15;
    double a1=0.03,a2=0.01,a3=0.03,a4=0.01;

    Particle[] P = new Particle[N];
    Random rng = new Random(1);

    MCL(GridMap m) {
      this.m = m;
      for (int i=0;i<N;i++) P[i]=new Particle();
      for (int i=0;i<13;i++){
        double deg = -90.0 + 180.0 * ((double)i/12.0);
        beams[i] = deg * PI / 180.0;
      }
    }

    void initUniform() {
      ArrayList<int[]> free = new ArrayList<>();
      for (int y=0;y<m.H;y++)
        for (int x=0;x<m.W;x++)
          if (m.at(x,y)==0) free.add(new int[]{x,y});
      for (int i=0;i<N;i++){
        int[] c = free.get(rng.nextInt(free.size()));
        P[i].x = (c[0] + 0.5) * m.res;
        P[i].y = (c[1] + 0.5) * m.res;
        P[i].th = -PI + 2.0*PI*rng.nextDouble();
        P[i].w = 1.0/N;
      }
    }

    void motionUpdate(double dRot1,double dTrans,double dRot2) {
      double sr1 = Math.sqrt(a1*dRot1*dRot1 + a2*dTrans*dTrans);
      double st  = Math.sqrt(a3*dTrans*dTrans + a4*(dRot1*dRot1 + dRot2*dRot2));
      double sr2 = Math.sqrt(a1*dRot2*dRot2 + a2*dTrans*dTrans);
      int[] g = new int[2];

      for (int i=0;i<N;i++){
        Particle p = P[i];
        double dr1 = dRot1 + sr1*rng.nextGaussian();
        double dt  = dTrans + st*rng.nextGaussian();
        double dr2 = dRot2 + sr2*rng.nextGaussian();

        double xn = p.x + dt*Math.cos(p.th + dr1);
        double yn = p.y + dt*Math.sin(p.th + dr1);
        double thn = wrap(p.th + dr1 + dr2);

        m.worldToGrid(xn, yn, g);
        if (m.at(g[0], g[1])==0) { p.x=xn; p.y=yn; p.th=thn; }
      }
    }

    void sensorUpdate(double[] z) {
      double sigma = sigmaHit;
      double invs2 = 1.0/(sigma*sigma);
      double norm  = 1.0/(Math.sqrt(2.0*PI)*sigma);
      double unif  = 1.0/rMax;

      double[] logw = new double[N];
      for (int bi=0; bi<beams.length; bi++){
        for (int i=0;i<N;i++){
          Particle p = P[i];
          double zexp = rayCast(m, p.x, p.y, p.th + beams[bi], rMax, 0.02);
          double dz = z[bi] - zexp;
          double prob = zHit*norm*Math.exp(-0.5*dz*dz*invs2) + zRand*unif;
          logw[i] += Math.log(prob + 1e-12);
        }
      }
      double mx = logw[0];
      for (int i=1;i<N;i++) mx = Math.max(mx, logw[i]);

      double sum = 0.0;
      double[] w = new double[N];
      for (int i=0;i<N;i++){ w[i]=Math.exp(logw[i]-mx); sum += w[i]; }
      for (int i=0;i<N;i++) P[i].w = (sum>0)?(w[i]/sum):(1.0/N);
    }

    double neff() {
      double s = 0.0;
      for (int i=0;i<N;i++) s += P[i].w * P[i].w;
      return 1.0/s;
    }

    void resampleIfNeeded(double ratio, double inject) {
      if (neff() >= ratio*N) return;

      double[] w = new double[N];
      for (int i=0;i<N;i++) w[i]=P[i].w;
      int[] idx = systematicResample(w, rng);
      Particle[] P2 = new Particle[N];
      for (int i=0;i<N;i++){
        Particle src = P[idx[i]];
        Particle dst = new Particle();
        dst.x=src.x; dst.y=src.y; dst.th=src.th; dst.w=1.0/N;
        P2[i]=dst;
      }
      P = P2;

      int k = (int)Math.round(inject*N);
      if (k<=0) return;

      ArrayList<int[]> free = new ArrayList<>();
      for (int y=0;y<m.H;y++)
        for (int x=0;x<m.W;x++)
          if (m.at(x,y)==0) free.add(new int[]{x,y});

      for (int i=0;i<k;i++){
        int[] c = free.get(rng.nextInt(free.size()));
        P[i].x = (c[0] + 0.5) * m.res;
        P[i].y = (c[1] + 0.5) * m.res;
        P[i].th = -PI + 2.0*PI*rng.nextDouble();
      }
    }

    double[] estimate() {
      double mx=0,my=0,s=0,c=0;
      for (int i=0;i<N;i++){
        Particle p=P[i];
        mx += p.w*p.x; my += p.w*p.y;
        s  += p.w*Math.sin(p.th); c += p.w*Math.cos(p.th);
      }
      double th = Math.atan2(s,c);
      return new double[]{mx,my,th};
    }
  }

  static void simulateStep(double[] p, double v, double w, double dt){
    p[0] += v*Math.cos(p[2])*dt;
    p[1] += v*Math.sin(p[2])*dt;
    p[2] = wrap(p[2] + w*dt);
  }

  static double[] odomFromTrue(double[] p1, double[] p2){
    double dx = p2[0]-p1[0], dy = p2[1]-p1[1];
    double dT = Math.sqrt(dx*dx + dy*dy);
    double dir = Math.atan2(dy, dx);
    double dR1 = (dT>1e-9) ? wrap(dir - p1[2]) : 0.0;
    double dR2 = wrap(p2[2] - p1[2] - dR1);
    return new double[]{dR1, dT, dR2};
  }

  static double[] rangeScan(GridMap m, double[] p, double[] beams, double rMax, double noiseSigma, Random rng){
    double[] z = new double[beams.length];
    for (int i=0;i<beams.length;i++){
      double r = rayCast(m, p[0], p[1], p[2] + beams[i], rMax, 0.02);
      r += noiseSigma*rng.nextGaussian();
      if (r < 0) r = 0;
      if (r > rMax) r = rMax;
      z[i]=r;
    }
    return z;
  }

  public static void main(String[] args) throws Exception {
    GridMap m = new GridMap();
    m.buildDemo();
    MCL pf = new MCL(m);
    pf.initUniform();

    double[] truePose = new double[]{2.0,2.0,0.0};
    double dt = 0.25;
    int T = 180;
    double vCmd=0.35, wCmd=0.25;
    Random rng = new Random(7);

    try (PrintWriter fTrue = new PrintWriter(new FileWriter("true_traj.csv"));
         PrintWriter fEst  = new PrintWriter(new FileWriter("est_traj.csv"))) {
      fTrue.println("t,x,y,theta");
      fEst.println("t,x,y,theta");
      int[] g = new int[2];

      for (int t=0;t<T;t++){
        double[] next = Arrays.copyOf(truePose, 3);
        simulateStep(next, vCmd, wCmd, dt);
        m.worldToGrid(next[0], next[1], g);
        if (m.at(g[0], g[1]) != 0) {
          next = Arrays.copyOf(truePose, 3);
          simulateStep(next, 0.0, 0.8, dt);
        }

        double[] odom = odomFromTrue(truePose, next);
        double[] z = rangeScan(m, next, pf.beams, pf.rMax, 0.05, rng);

        pf.motionUpdate(odom[0], odom[1], odom[2]);
        pf.sensorUpdate(z);
        pf.resampleIfNeeded(0.55, 0.03);

        double[] est = pf.estimate();
        truePose = next;

        fTrue.printf(Locale.US, "%d,%.6f,%.6f,%.6f%n", t, truePose[0], truePose[1], truePose[2]);
        fEst.printf(Locale.US, "%d,%.6f,%.6f,%.6f%n", t, est[0], est[1], est[2]);
      }
    }
    System.out.println("Wrote true_traj.csv and est_traj.csv");
  }
}
