// Chapter12_Lesson3.java
// Pose Graph Optimization in SE(2) via Gauss-Newton (educational, EJML)
//
// Maven (pom.xml) dependency (EJML):
//   <dependency>
//     <groupId>org.ejml</groupId>
//     <artifactId>ejml-all</artifactId>
//     <version>0.43</version>
//   </dependency>
//
// Compile/run (example):
//   mvn -q -DskipTests package
//   java -cp target/classes:~/.m2/repository/... Chapter12_Lesson3
//
// Notes:
// - This implementation uses dense matrices for clarity (small graphs).
// - For large graphs, use sparse libraries or call into C++/native solvers.

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import java.util.ArrayList;
import java.util.Random;

public class Chapter12_Lesson3 {

    static class EdgeSE2 {
        int i, j;
        double[] z;         // [dx, dy, dtheta] measurement
        DMatrixRMaj Omega;  // 3x3 information

        EdgeSE2(int i, int j, double[] z, DMatrixRMaj Omega) {
            this.i = i; this.j = j; this.z = z; this.Omega = Omega;
        }
    }

    static double wrapAngle(double a) {
        a = (a + Math.PI) % (2.0 * Math.PI);
        if (a < 0) a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    static double[][] rot2(double th) {
        double c = Math.cos(th), s = Math.sin(th);
        return new double[][]{{c, -s}, {s, c}};
    }

    static double[] predictRelative(double[] xi, double[] xj) {
        double[] ti = {xi[0], xi[1]};
        double[] tj = {xj[0], xj[1]};
        double[][] Ri = rot2(xi[2]);

        // dt = Ri^T (tj - ti)
        double dx = tj[0] - ti[0];
        double dy = tj[1] - ti[1];
        double dt0 = Ri[0][0]*dx + Ri[1][0]*dy;
        double dt1 = Ri[0][1]*dx + Ri[1][1]*dy;

        double dth = wrapAngle(xj[2] - xi[2]);
        return new double[]{dt0, dt1, dth};
    }

    static double[] errorBetween(double[] xi, double[] xj, double[] z) {
        double[] zhat = predictRelative(xi, xj);
        double[][] Rz = rot2(z[2]);

        // terr = Rz^T (zhat_t - z_t)
        double ex = zhat[0] - z[0];
        double ey = zhat[1] - z[1];
        double terr0 = Rz[0][0]*ex + Rz[1][0]*ey;
        double terr1 = Rz[0][1]*ex + Rz[1][1]*ey;

        double therr = wrapAngle(zhat[2] - z[2]);
        return new double[]{terr0, terr1, therr};
    }

    static void jacobians(double[] xi, double[] xj, double[] z, DMatrixRMaj A, DMatrixRMaj B) {
        // A,B are 3x3
        CommonOps_DDRM.fill(A, 0.0);
        CommonOps_DDRM.fill(B, 0.0);

        double[] ti = {xi[0], xi[1]};
        double[] tj = {xj[0], xj[1]};
        double[][] Ri = rot2(xi[2]);
        double[][] Rz = rot2(z[2]);

        // A2 = Rz^T Ri^T
        // We'll build Ri^T then multiply by Rz^T
        double[][] Rit = {{Ri[0][0], Ri[1][0]}, {Ri[0][1], Ri[1][1]}};
        double[][] Rzt = {{Rz[0][0], Rz[1][0]}, {Rz[0][1], Rz[1][1]}};

        double a00 = Rzt[0][0]*Rit[0][0] + Rzt[0][1]*Rit[1][0];
        double a01 = Rzt[0][0]*Rit[0][1] + Rzt[0][1]*Rit[1][1];
        double a10 = Rzt[1][0]*Rit[0][0] + Rzt[1][1]*Rit[1][0];
        double a11 = Rzt[1][0]*Rit[0][1] + Rzt[1][1]*Rit[1][1];

        // translation derivatives
        // A[0:2,0:2] = -A2, B[0:2,0:2] = A2
        A.set(0,0, -a00); A.set(0,1, -a01);
        A.set(1,0, -a10); A.set(1,1, -a11);

        B.set(0,0,  a00); B.set(0,1,  a01);
        B.set(1,0,  a10); B.set(1,1,  a11);

        // d(Ri^T (tj-ti))/dtheta_i = -Ri^T S (tj-ti)
        // S = [[0,-1],[1,0]]
        double dx = tj[0] - ti[0];
        double dy = tj[1] - ti[1];

        // S*dt
        double sdx = -dy;
        double sdy =  dx;

        // Ri^T (S*dt)
        double v0 = Rit[0][0]*sdx + Rit[0][1]*sdy;
        double v1 = Rit[1][0]*sdx + Rit[1][1]*sdy;

        // -Ri^T(S*dt)
        v0 = -v0; v1 = -v1;

        // Rz^T * v
        double d0 = Rzt[0][0]*v0 + Rzt[0][1]*v1;
        double d1 = Rzt[1][0]*v0 + Rzt[1][1]*v1;

        A.set(0,2, d0);
        A.set(1,2, d1);

        // theta derivatives
        A.set(2,2, -1.0);
        B.set(2,2,  1.0);
    }

    static double[][] gaussNewton(double[][] X0, ArrayList<EdgeSE2> edges, int iters) {
        int N = X0.length;
        double[][] X = new double[N][3];
        for(int k=0;k<N;k++) System.arraycopy(X0[k],0,X[k],0,3);

        // anchor pose 0, variables are poses 1..N-1 => dim = 3*(N-1)
        int dim = 3*(N-1);

        for(int it=0; it<iters; it++) {
            DMatrixRMaj H = new DMatrixRMaj(dim, dim);
            DMatrixRMaj b = new DMatrixRMaj(dim, 1);

            double chi2 = 0.0;

            for(EdgeSE2 e: edges) {
                double[] xi = X[e.i];
                double[] xj = X[e.j];
                double[] r = errorBetween(xi, xj, e.z);

                // chi2
                DMatrixRMaj rM = new DMatrixRMaj(3,1,true, r);
                DMatrixRMaj tmp = new DMatrixRMaj(3,1);
                CommonOps_DDRM.mult(e.Omega, rM, tmp);
                chi2 += CommonOps_DDRM.dot(rM, tmp);

                DMatrixRMaj A = new DMatrixRMaj(3,3);
                DMatrixRMaj Bm = new DMatrixRMaj(3,3);
                jacobians(xi, xj, e.z, A, Bm);

                // local contributions
                // Hii = A^T Omega A, Hij = A^T Omega B, etc.
                DMatrixRMaj At = new DMatrixRMaj(3,3);
                DMatrixRMaj Bt = new DMatrixRMaj(3,3);
                CommonOps_DDRM.transpose(A, At);
                CommonOps_DDRM.transpose(Bm, Bt);

                DMatrixRMaj AtOmega = new DMatrixRMaj(3,3);
                DMatrixRMaj BtOmega = new DMatrixRMaj(3,3);
                CommonOps_DDRM.mult(At, e.Omega, AtOmega);
                CommonOps_DDRM.mult(Bt, e.Omega, BtOmega);

                DMatrixRMaj Hii = new DMatrixRMaj(3,3);
                DMatrixRMaj Hij = new DMatrixRMaj(3,3);
                DMatrixRMaj Hji = new DMatrixRMaj(3,3);
                DMatrixRMaj Hjj = new DMatrixRMaj(3,3);

                CommonOps_DDRM.mult(AtOmega, A, Hii);
                CommonOps_DDRM.mult(AtOmega, Bm, Hij);
                CommonOps_DDRM.mult(BtOmega, A, Hji);
                CommonOps_DDRM.mult(BtOmega, Bm, Hjj);

                DMatrixRMaj bi = new DMatrixRMaj(3,1);
                DMatrixRMaj bj = new DMatrixRMaj(3,1);
                CommonOps_DDRM.mult(AtOmega, rM, bi);
                CommonOps_DDRM.mult(BtOmega, rM, bj);

                // scatter (skip anchored pose 0)
                // index mapping: pose k -> base = 3*(k-1)
                java.util.function.IntUnaryOperator idx = (pid) -> 3*(pid-1);

                // add to H
                addBlockDense(H, Hii, e.i, e.i, idx);
                addBlockDense(H, Hij, e.i, e.j, idx);
                addBlockDense(H, Hji, e.j, e.i, idx);
                addBlockDense(H, Hjj, e.j, e.j, idx);

                // add to b
                if(e.i != 0) addVecDense(b, bi, e.i, idx);
                if(e.j != 0) addVecDense(b, bj, e.j, idx);
            }

            // Solve H dx = -b using EJML (dense)
            DMatrixRMaj minusb = b.copy();
            CommonOps_DDRM.scale(-1.0, minusb);

            DMatrixRMaj dx = new DMatrixRMaj(dim,1);
            CommonOps_DDRM.solve(H, minusb, dx);

            // apply
            double normdx = 0.0;
            for(int k=1;k<N;k++){
                int base = 3*(k-1);
                double d0 = dx.get(base,0);
                double d1 = dx.get(base+1,0);
                double d2 = dx.get(base+2,0);
                X[k][0] += d0; X[k][1] += d1; X[k][2] = wrapAngle(X[k][2] + d2);
                normdx += d0*d0 + d1*d1 + d2*d2;
            }
            normdx = Math.sqrt(normdx);
            System.out.printf("iter=%02d  chi2=%.6f  |dx|=%.3e%n", it, chi2, normdx);
            if(normdx < 1e-8) break;
        }
        return X;
    }

    static void addBlockDense(DMatrixRMaj H, DMatrixRMaj M, int p, int q, java.util.function.IntUnaryOperator idx) {
        if(p==0 || q==0) return; // anchored
        int ip = idx.applyAsInt(p);
        int iq = idx.applyAsInt(q);
        for(int a=0;a<3;a++){
            for(int c=0;c<3;c++){
                H.add(ip+a, iq+c, M.get(a,c));
            }
        }
    }

    static void addVecDense(DMatrixRMaj b, DMatrixRMaj v, int p, java.util.function.IntUnaryOperator idx) {
        int ip = idx.applyAsInt(p);
        for(int a=0;a<3;a++){
            b.add(ip+a, 0, v.get(a,0));
        }
    }

    static void makeSynthetic(int N, double[][] X0, ArrayList<EdgeSE2> edges) {
        double[][] Xtrue = new double[N][3];
        for(int k=1;k<N;k++){
            Xtrue[k][0] = Xtrue[k-1][0] + 0.5*Math.cos(0.1*k);
            Xtrue[k][1] = Xtrue[k-1][1] + 0.5*Math.sin(0.1*k);
            Xtrue[k][2] = wrapAngle(0.05*k);
        }

        double noise_xy = 0.02, noise_th = 0.01;
        DMatrixRMaj Sigma = CommonOps_DDRM.identity(3);
        Sigma.set(0,0, noise_xy*noise_xy);
        Sigma.set(1,1, noise_xy*noise_xy);
        Sigma.set(2,2, noise_th*noise_th);
        DMatrixRMaj Omega = new DMatrixRMaj(3,3);
        CommonOps_DDRM.invert(Sigma, Omega);

        Random rng = new Random(7);

        edges.clear();
        for(int k=0;k<N-1;k++){
            double[] z = predictRelative(Xtrue[k], Xtrue[k+1]);
            z[0] += noise_xy * rng.nextGaussian();
            z[1] += noise_xy * rng.nextGaussian();
            z[2]  = wrapAngle(z[2] + noise_th * rng.nextGaussian());
            edges.add(new EdgeSE2(k, k+1, z, Omega));
        }

        // loop closure 0->N-1
        double[] zlc = predictRelative(Xtrue[0], Xtrue[N-1]);
        zlc[0] += noise_xy * rng.nextGaussian();
        zlc[1] += noise_xy * rng.nextGaussian();
        zlc[2]  = wrapAngle(zlc[2] + noise_th * rng.nextGaussian());
        edges.add(new EdgeSE2(0, N-1, zlc, Omega));

        // init from odometry
        for(int k=0;k<N;k++){ X0[k][0]=0; X0[k][1]=0; X0[k][2]=0; }
        for(int k=0;k<N-1;k++){
            double[] z = edges.get(k).z;
            double th = X0[k][2];
            double[][] Rk = rot2(th);
            X0[k+1][0] = X0[k][0] + (Rk[0][0]*z[0] + Rk[0][1]*z[1]);
            X0[k+1][1] = X0[k][1] + (Rk[1][0]*z[0] + Rk[1][1]*z[1]);
            X0[k+1][2] = wrapAngle(X0[k][2] + z[2]);
        }
    }

    public static void main(String[] args) {
        int N = 25;
        double[][] X0 = new double[N][3];
        ArrayList<EdgeSE2> edges = new ArrayList<>();
        makeSynthetic(N, X0, edges);

        System.out.println("Optimizing pose graph (Gauss-Newton, anchor pose 0)...");
        double[][] Xopt = gaussNewton(X0, edges, 12);

        System.out.println("\nFirst 5 poses (x,y,theta) initial vs optimized:");
        for(int k=0;k<5;k++){
            System.out.printf("k=%02d  X0=[%.3f %.3f %.3f]  Xopt=[%.3f %.3f %.3f]%n",
                    k, X0[k][0], X0[k][1], X0[k][2], Xopt[k][0], Xopt[k][1], Xopt[k][2]);
        }
    }
}
