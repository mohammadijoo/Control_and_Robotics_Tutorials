// Chapter12_Lesson1.java
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 12 — SLAM II: Graph-Based SLAM
Lesson 1 — Pose Graphs and Factor Graphs

Minimal 2D pose graph Gauss–Newton optimizer (SE(2)) using EJML (dense).

Build (example with Maven/Gradle dependency on EJML):
  org.ejml:ejml-all:0.43+

This file is written as a single-class demo for teaching.
*/

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import java.util.*;

public class Chapter12_Lesson1 {

    static class Pose2 {
        double x, y, th;
        Pose2(double x, double y, double th) { this.x=x; this.y=y; this.th=th; }
        Pose2 copy() { return new Pose2(x,y,th); }
    }

    static class Edge {
        int i, j;
        double[] z;      // length 3
        DMatrixRMaj Omega; // 3x3
        Edge(int i, int j, double[] z, DMatrixRMaj Omega) { this.i=i; this.j=j; this.z=z; this.Omega=Omega; }
    }

    static double wrapAngle(double a) {
        a = (a + Math.PI) % (2.0*Math.PI);
        if (a &lt; 0) a += 2.0*Math.PI;
        return a - Math.PI;
    }

    static double[][] rot(double th) {
        double c = Math.cos(th), s = Math.sin(th);
        return new double[][]{
                {c, -s},
                {s,  c}
        };
    }

    static Pose2 boxPlus(Pose2 p, double[] d) {
        double[][] R = rot(p.th);
        double dx = R[0][0]*d[0] + R[0][1]*d[1];
        double dy = R[1][0]*d[0] + R[1][1]*d[1];
        return new Pose2(p.x + dx, p.y + dy, wrapAngle(p.th + d[2]));
    }

    static double[] between(Pose2 i, Pose2 j) {
        double dx = j.x - i.x;
        double dy = j.y - i.y;

        double[][] RiT = rot(i.th); // we will use transpose explicitly
        // transpose(R) * dt:
        double dxi =  RiT[0][0]*dx + RiT[1][0]*dy;
        double dyi =  RiT[0][1]*dx + RiT[1][1]*dy;

        double dth = wrapAngle(j.th - i.th);
        return new double[]{dxi, dyi, dth};
    }

    static class LinEdge {
        double[] r;      // 3
        DMatrixRMaj Ji;  // 3x3
        DMatrixRMaj Jj;  // 3x3
        LinEdge(double[] r, DMatrixRMaj Ji, DMatrixRMaj Jj) { this.r=r; this.Ji=Ji; this.Jj=Jj; }
    }

    static LinEdge residualAndJacobians(Pose2 xi, Pose2 xj, double[] z) {
        double[] zhat = between(xi, xj);
        double[] r = new double[]{ z[0]-zhat[0], z[1]-zhat[1], wrapAngle(z[2]-zhat[2]) };

        // Prepare matrices
        DMatrixRMaj Ji = new DMatrixRMaj(3,3);
        DMatrixRMaj Jj = new DMatrixRMaj(3,3);

        // RiT
        double c = Math.cos(xi.th), s = Math.sin(xi.th);
        double r00 = c, r01 = s;
        double r10 = -s, r11 = c;

        // dt
        double dtx = xj.x - xi.x;
        double dty = xj.y - xi.y;

        // J = [[0,-1],[1,0]]
        double j00 = 0, j01 = -1;
        double j10 = 1, j11 = 0;

        // dri/dti = RiT
        Ji.set(0,0, r00); Ji.set(0,1, r01);
        Ji.set(1,0, r10); Ji.set(1,1, r11);

        // dri/dtheta = RiT * J * dt
        double Jdt_x = j00*dtx + j01*dty;
        double Jdt_y = j10*dtx + j11*dty;
        double dri_dth_x = r00*Jdt_x + r01*Jdt_y;
        double dri_dth_y = r10*Jdt_x + r11*Jdt_y;
        Ji.set(0,2, dri_dth_x);
        Ji.set(1,2, dri_dth_y);

        // dr_theta/dtheta_i = +1
        Ji.set(2,2, 1.0);

        // dr/dt_j = -RiT
        Jj.set(0,0, -r00); Jj.set(0,1, -r01);
        Jj.set(1,0, -r10); Jj.set(1,1, -r11);

        // dr_theta/dtheta_j = -1
        Jj.set(2,2, -1.0);

        return new LinEdge(r, Ji, Jj);
    }

    static void addSubMatrix(DMatrixRMaj A, int r0, int c0, DMatrixRMaj B) {
        for (int r=0; r&lt;B.numRows; r++) {
            for (int c=0; c&lt;B.numCols; c++) {
                A.add(r0+r, c0+c, B.get(r,c));
            }
        }
    }

    static void addSubVector(DMatrixRMaj v, int r0, double[] b) {
        for (int r=0; r&lt;b.length; r++) v.add(r0+r, 0, b[r]);
    }

    static void buildNormalEquations(List&lt;Pose2&gt; poses, List&lt;Edge&gt; edges, List&lt;Edge&gt; priors,
                                     DMatrixRMaj H, DMatrixRMaj g)
    {
        int N = poses.size();
        CommonOps_DDRM.fill(H, 0);
        CommonOps_DDRM.fill(g, 0);

        for (Edge e : edges) {
            LinEdge lin = residualAndJacobians(poses.get(e.i), poses.get(e.j), e.z);

            // Hi = Ji^T Omega Ji
            DMatrixRMaj tmp = new DMatrixRMaj(3,3);
            DMatrixRMaj Hi = new DMatrixRMaj(3,3);
            DMatrixRMaj Hj = new DMatrixRMaj(3,3);
            DMatrixRMaj Hij = new DMatrixRMaj(3,3);

            // tmp = Omega * Ji
            CommonOps_DDRM.mult(e.Omega, lin.Ji, tmp);
            CommonOps_DDRM.multTransA(lin.Ji, tmp, Hi);

            CommonOps_DDRM.mult(e.Omega, lin.Jj, tmp);
            CommonOps_DDRM.multTransA(lin.Jj, tmp, Hj);

            CommonOps_DDRM.mult(e.Omega, lin.Jj, tmp);
            CommonOps_DDRM.multTransA(lin.Ji, tmp, Hij);

            // gi = Ji^T Omega r
            DMatrixRMaj rvec = new DMatrixRMaj(3,1, true, lin.r);
            DMatrixRMaj Or = new DMatrixRMaj(3,1);
            CommonOps_DDRM.mult(e.Omega, rvec, Or);

            DMatrixRMaj gi = new DMatrixRMaj(3,1);
            DMatrixRMaj gj = new DMatrixRMaj(3,1);
            CommonOps_DDRM.multTransA(lin.Ji, Or, gi);
            CommonOps_DDRM.multTransA(lin.Jj, Or, gj);

            int si = 3*e.i, sj = 3*e.j;
            addSubMatrix(H, si, si, Hi);
            addSubMatrix(H, sj, sj, Hj);
            addSubMatrix(H, si, sj, Hij);

            // add Hij^T into (sj,si)
            DMatrixRMaj HijT = new DMatrixRMaj(3,3);
            CommonOps_DDRM.transpose(Hij, HijT);
            addSubMatrix(H, sj, si, HijT);

            // g
            addSubVector(g, si, gi.getData());
            addSubVector(g, sj, gj.getData());
        }

        // Priors: r = mu - x, J = -I
        for (Edge p : priors) {
            int i = p.i;
            Pose2 xi = poses.get(i);
            double[] mu = p.z;
            double[] r = new double[]{ mu[0]-xi.x, mu[1]-xi.y, wrapAngle(mu[2]-xi.th) };

            DMatrixRMaj J = CommonOps_DDRM.identity(3);
            CommonOps_DDRM.scale(-1.0, J);

            DMatrixRMaj tmp = new DMatrixRMaj(3,3);
            DMatrixRMaj Hi = new DMatrixRMaj(3,3);
            CommonOps_DDRM.mult(p.Omega, J, tmp);
            CommonOps_DDRM.multTransA(J, tmp, Hi);

            DMatrixRMaj rvec = new DMatrixRMaj(3,1, true, r);
            DMatrixRMaj Or = new DMatrixRMaj(3,1);
            CommonOps_DDRM.mult(p.Omega, rvec, Or);
            DMatrixRMaj gi = new DMatrixRMaj(3,1);
            CommonOps_DDRM.multTransA(J, Or, gi);

            int si = 3*i;
            addSubMatrix(H, si, si, Hi);
            addSubVector(g, si, gi.getData());
        }
    }

    static List&lt;Pose2&gt; gaussNewton(List&lt;Pose2&gt; x0, List&lt;Edge&gt; edges, List&lt;Edge&gt; priors, int iters) {
        int N = x0.size();
        List&lt;Pose2&gt; x = new ArrayList&lt;&gt;();
        for (Pose2 p : x0) x.add(p.copy());

        DMatrixRMaj H = new DMatrixRMaj(3*N, 3*N);
        DMatrixRMaj g = new DMatrixRMaj(3*N, 1);

        for (int k=0; k&lt;iters; k++) {
            buildNormalEquations(x, edges, priors, H, g);

            // Damping
            for (int d=0; d&lt;3*N; d++) H.add(d,d, 1e-8);

            // Solve H dx = -g
            DMatrixRMaj minusG = g.copy();
            CommonOps_DDRM.scale(-1.0, minusG);
            DMatrixRMaj dx = new DMatrixRMaj(3*N, 1);
            CommonOps_DDRM.solve(H, minusG, dx);

            double maxStep = 0.0;
            for (int i=0; i&lt;N; i++) {
                double[] d = new double[]{ dx.get(3*i,0), dx.get(3*i+1,0), dx.get(3*i+2,0) };
                Pose2 upd = boxPlus(x.get(i), d);
                x.set(i, upd);
                double n = Math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
                if (n &gt; maxStep) maxStep = n;
            }
            System.out.println("iter " + k + ": max|delta|=" + maxStep);
            if (maxStep &lt; 1e-9) break;
        }
        return x;
    }

    public static void main(String[] args) {
        // Ground truth
        List&lt;Pose2&gt; gt = Arrays.asList(
                new Pose2(0,0,0),
                new Pose2(1,0,0),
                new Pose2(2,0,0),
                new Pose2(2,1,Math.PI/2),
                new Pose2(2,2,Math.PI/2),
                new Pose2(1,2,Math.PI),
                new Pose2(0,2,Math.PI),
                new Pose2(0,1,-Math.PI/2)
        );
        int N = gt.size();
        Random rng = new Random(4);

        double sigmaXY = 0.02;
        double sigmaTh = Math.toRadians(1.0);
        DMatrixRMaj Omega = new DMatrixRMaj(3,3);
        Omega.set(0,0, 1.0/(sigmaXY*sigmaXY));
        Omega.set(1,1, 1.0/(sigmaXY*sigmaXY));
        Omega.set(2,2, 1.0/(sigmaTh*sigmaTh));

        List&lt;Edge&gt; edges = new ArrayList&lt;&gt;();
        for (int i=0; i&lt;N-1; i++) {
            double[] z = between(gt.get(i), gt.get(i+1));
            z[0] += rng.nextGaussian()*sigmaXY;
            z[1] += rng.nextGaussian()*sigmaXY;
            z[2] = wrapAngle(z[2] + rng.nextGaussian()*sigmaTh);
            edges.add(new Edge(i, i+1, z, Omega));
        }
        // loop closure 7->0
        {
            double[] z = between(gt.get(7), gt.get(0));
            z[0] += rng.nextGaussian()*sigmaXY;
            z[1] += rng.nextGaussian()*sigmaXY;
            z[2] = wrapAngle(z[2] + rng.nextGaussian()*sigmaTh);
            edges.add(new Edge(7, 0, z, Omega));
        }

        // Initial guess
        List&lt;Pose2&gt; x0 = new ArrayList&lt;&gt;();
        for (Pose2 p : gt) {
            x0.add(new Pose2(
                    p.x + rng.nextGaussian()*0.15,
                    p.y + rng.nextGaussian()*0.15,
                    wrapAngle(p.th + rng.nextGaussian()*Math.toRadians(5.0))
            ));
        }

        // Prior on node 0
        DMatrixRMaj Omega0 = new DMatrixRMaj(3,3);
        Omega0.set(0,0, 1e6); Omega0.set(1,1, 1e6); Omega0.set(2,2, 1e6);
        List&lt;Edge&gt; priors = new ArrayList&lt;&gt;();
        priors.add(new Edge(0, 0, new double[]{gt.get(0).x, gt.get(0).y, gt.get(0).th}, Omega0));

        System.out.println("Optimizing...");
        List&lt;Pose2&gt; est = gaussNewton(x0, edges, priors, 15);

        System.out.println("\n i | gt(x,y,th) | est(x,y,th)");
        for (int i=0; i&lt;N; i++) {
            Pose2 gtp = gt.get(i), ep = est.get(i);
            System.out.printf("%d | (%.3f,%.3f,%.3f) | (%.3f,%.3f,%.3f)\n",
                    i, gtp.x, gtp.y, gtp.th, ep.x, ep.y, ep.th);
        }
    }
}
