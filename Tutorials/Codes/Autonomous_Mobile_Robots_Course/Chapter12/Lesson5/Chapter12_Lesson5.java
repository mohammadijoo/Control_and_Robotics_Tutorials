// Chapter12_Lesson5.java
// Lab: Build and Optimize a 2D Pose Graph (SE(2)) using Gauss-Newton / LM
//
// Dependencies: EJML (Efficient Java Matrix Library)
// Maven (example):
//   <dependency>
//     <groupId>org.ejml</groupId>
//     <artifactId>ejml-all</artifactId>
//     <version>0.43</version>
//   </dependency>
//
// This is an educational implementation. For large graphs you want sparse solvers;
// here we use dense matrices to keep the code self-contained and readable.

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter12_Lesson5 {

    static double wrapAngle(double a) {
        a = (a + Math.PI) % (2.0 * Math.PI) - Math.PI;
        if (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    static double[][] rot2(double th) {
        double c = Math.cos(th), s = Math.sin(th);
        return new double[][]{{c, -s}, {s, c}};
    }

    static double[] mat2Tvec(double[][] R, double[] v) {
        return new double[]{
                R[0][0] * v[0] + R[1][0] * v[1], // R^T v
                R[0][1] * v[0] + R[1][1] * v[1]
        };
    }

    static double[] mat2Vec(double[][] R, double[] v) {
        return new double[]{
                R[0][0] * v[0] + R[0][1] * v[1],
                R[1][0] * v[0] + R[1][1] * v[1]
        };
    }

    static final double[][] SKEW = new double[][]{{0, -1}, {1, 0}};

    static double[] skewMul(double[] v) {
        return new double[]{
                SKEW[0][0] * v[0] + SKEW[0][1] * v[1],
                SKEW[1][0] * v[0] + SKEW[1][1] * v[1]
        };
    }

    static class EdgeSE2 {
        int i, j;
        double[] z;        // [dx, dy, dtheta]
        double[][] Omega;  // 3x3
        EdgeSE2(int i, int j, double[] z, double[][] Omega) { this.i=i; this.j=j; this.z=z; this.Omega=Omega; }
    }

    static double[] se2Compose(double[] x, double[] y) {
        double[][] R = rot2(x[2]);
        double[] t = mat2Vec(R, new double[]{y[0], y[1]});
        return new double[]{ x[0] + t[0], x[1] + t[1], wrapAngle(x[2] + y[2]) };
    }

    static double[] se2Inverse(double[] x) {
        double[][] R = rot2(x[2]);
        double[] tinv = mat2Tvec(R, new double[]{-x[0], -x[1]});
        return new double[]{ tinv[0], tinv[1], wrapAngle(-x[2]) };
    }

    static double[] se2Between(double[] xi, double[] xj) {
        return se2Compose(se2Inverse(xi), xj);
    }

    static double[] errorAndJacobians(double[] xi, double[] xj, double[] z, double[][] A, double[][] B) {
        double[] ti = new double[]{xi[0], xi[1]};
        double[] tj = new double[]{xj[0], xj[1]};
        double thi = xi[2], thj = xj[2];

        double[][] Ri = rot2(thi);
        double[][] Rz = rot2(z[2]);

        double[] dt = new double[]{tj[0]-ti[0], tj[1]-ti[1]};
        double[] zhat_t = mat2Tvec(Ri, dt);
        double zhat_th = wrapAngle(thj - thi);

        double[] diff = new double[]{zhat_t[0]-z[0], zhat_t[1]-z[1]};
        double[] e_t = mat2Tvec(Rz, diff);
        double e_th = wrapAngle(zhat_th - z[2]);

        // A,B initialized by caller
        for(int r=0;r<3;r++){ for(int c=0;c<3;c++){ A[r][c]=0; B[r][c]=0; } }

        // A_pos = -Rz^T Ri^T ; B_pos = +Rz^T Ri^T
        double[][] RzT = new double[][]{{Rz[0][0], Rz[1][0]}, {Rz[0][1], Rz[1][1]}};
        double[][] RiT = new double[][]{{Ri[0][0], Ri[1][0]}, {Ri[0][1], Ri[1][1]}};

        double[][] M = mul2(RzT, RiT);
        A[0][0] = -M[0][0]; A[0][1] = -M[0][1];
        A[1][0] = -M[1][0]; A[1][1] = -M[1][1];
        B[0][0] =  M[0][0]; B[0][1] =  M[0][1];
        B[1][0] =  M[1][0]; B[1][1] =  M[1][1];

        // d(Ri^T dt)/dtheta = -Ri^T S dt
        double[] sdt = skewMul(dt);
        double[] Rit_sdt = mat2Tvec(Ri, sdt);
        double[] d_ri = new double[]{-Rit_sdt[0], -Rit_sdt[1]};

        double[] A_theta = mat2Tvec(Rz, d_ri);
        A[0][2] = A_theta[0];
        A[1][2] = A_theta[1];

        A[2][2] = -1.0;
        B[2][2] =  1.0;

        return new double[]{e_t[0], e_t[1], e_th};
    }

    static double[][] mul2(double[][] A, double[][] B) {
        return new double[][]{
                {A[0][0]*B[0][0] + A[0][1]*B[1][0], A[0][0]*B[0][1] + A[0][1]*B[1][1]},
                {A[1][0]*B[0][0] + A[1][1]*B[1][0], A[1][0]*B[0][1] + A[1][1]*B[1][1]}
        };
    }

    static double[][] invDiag(double sx, double sy, double sth) {
        double[][] Om = new double[3][3];
        Om[0][0] = 1.0/(sx*sx); Om[1][1] = 1.0/(sy*sy); Om[2][2] = 1.0/(sth*sth);
        return Om;
    }

    static void accumulate(DMatrixRMaj H, double[] b, int baseR, int baseC, double[][] M) {
        for(int r=0;r<3;r++){
            for(int c=0;c<3;c++){
                H.add(baseR+r, baseC+c, M[r][c]);
            }
        }
    }

    static double[] mat3Vec(double[][] M, double[] v) {
        return new double[]{
                M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
                M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
                M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
        };
    }

    static double[][] mat3Mul(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for(int r=0;r<3;r++){
            for(int c=0;c<3;c++){
                C[r][c] = A[r][0]*B[0][c] + A[r][1]*B[1][c] + A[r][2]*B[2][c];
            }
        }
        return C;
    }

    static double[][] mat3T(double[][] A) {
        double[][] T = new double[3][3];
        for(int r=0;r<3;r++) for(int c=0;c<3;c++) T[r][c]=A[c][r];
        return T;
    }

    static double[][] add3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for(int r=0;r<3;r++) for(int c=0;c<3;c++) C[r][c]=A[r][c]+B[r][c];
        return C;
    }

    static double[][] scale3(double[][] A, double s) {
        double[][] C = new double[3][3];
        for(int r=0;r<3;r++) for(int c=0;c<3;c++) C[r][c]=A[r][c]*s;
        return C;
    }

    static double dot3(double[] a, double[] b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

    static double cost(double[] x, int N, List<EdgeSE2> edges) {
        double c = 0.0;
        for(EdgeSE2 e: edges){
            double[] xi = new double[]{x[3*e.i], x[3*e.i+1], x[3*e.i+2]};
            double[] xj = new double[]{x[3*e.j], x[3*e.j+1], x[3*e.j+2]};
            double[] zhat = se2Between(xi, xj);
            double[] zinv = se2Inverse(e.z);
            double[] err = se2Compose(zinv, zhat);
            err[2] = wrapAngle(err[2]);
            double[] tmp = mat3Vec(e.Omega, err);
            c += dot3(err, tmp);
        }
        return c;
    }

    static void simulate(int N, long seed, double[] x0, double[] gt, List<EdgeSE2> edges) {
        Random rng = new Random(seed);

        // ground truth curve
        double[] cur = new double[]{0,0,0};
        for(int i=0;i<N;i++){
            gt[3*i] = cur[0]; gt[3*i+1]=cur[1]; gt[3*i+2]=cur[2];
            cur = se2Compose(cur, new double[]{0.5, 0.0, 0.03});
        }

        edges.clear();

        // odometry
        double[][] OmOdo = invDiag(0.05, 0.05, 0.02);
        for(int i=0;i<N-1;i++){
            double[] xi = new double[]{gt[3*i],gt[3*i+1],gt[3*i+2]};
            double[] xj = new double[]{gt[3*(i+1)],gt[3*(i+1)+1],gt[3*(i+1)+2]};
            double[] z = se2Between(xi, xj);
            z[0] += rng.nextGaussian()*0.05;
            z[1] += rng.nextGaussian()*0.05;
            z[2] = wrapAngle(z[2] + rng.nextGaussian()*0.02);
            edges.add(new EdgeSE2(i, i+1, z, OmOdo));
        }

        // loop closures
        double[][] OmLoop = invDiag(0.03, 0.03, 0.01);
        for(int i=0;i<N-12;i+=10){
            int j = i+10;
            double[] xi = new double[]{gt[3*i],gt[3*i+1],gt[3*i+2]};
            double[] xj = new double[]{gt[3*j],gt[3*j+1],gt[3*j+2]};
            double[] z = se2Between(xi, xj);
            z[0] += rng.nextGaussian()*0.03;
            z[1] += rng.nextGaussian()*0.03;
            z[2] = wrapAngle(z[2] + rng.nextGaussian()*0.01);
            edges.add(new EdgeSE2(i, j, z, OmLoop));
        }

        // initial guess by chaining odometry
        double[] c = new double[]{0,0,0};
        x0[0]=0; x0[1]=0; x0[2]=0;
        for(int i=0;i<N-1;i++){
            c = se2Compose(c, edges.get(i).z);
            x0[3*(i+1)] = c[0]; x0[3*(i+1)+1]=c[1]; x0[3*(i+1)+2]=c[2];
        }
    }

    static void rmse(double[] x, double[] gt, int N) {
        double sp=0, sa=0;
        for(int i=0;i<N;i++){
            double dx = x[3*i]-gt[3*i];
            double dy = x[3*i+1]-gt[3*i+1];
            sp += dx*dx + dy*dy;
            double da = wrapAngle(x[3*i+2]-gt[3*i+2]);
            sa += da*da;
        }
        System.out.printf("RMSE: pos=%.3f m, ang=%.3f rad%n", Math.sqrt(sp/N), Math.sqrt(sa/N));
    }

    public static void main(String[] args) {
        int N = 40; // dense solver -> keep moderate
        double[] x0 = new double[3*N];
        double[] gt = new double[3*N];
        List<EdgeSE2> edges = new ArrayList<>();
        simulate(N, 4L, x0, gt, edges);

        System.out.println("Initial:");
        rmse(x0, gt, N);

        double[] x = x0.clone();
        double cost0 = cost(x, N, edges);
        System.out.printf("iter 0: cost=%.6f%n", cost0);

        double lambda = 1e-3;
        for(int it=1; it<=20; it++){
            int dim = 3*N;
            DMatrixRMaj H = new DMatrixRMaj(dim, dim);
            double[] b = new double[dim];

            // build normal equations
            for(EdgeSE2 ed: edges){
                int i = ed.i, j = ed.j;
                double[] xi = new double[]{x[3*i],x[3*i+1],x[3*i+2]};
                double[] xj = new double[]{x[3*j],x[3*j+1],x[3*j+2]};
                double[][] A = new double[3][3];
                double[][] Bm = new double[3][3];
                double[] e = errorAndJacobians(xi, xj, ed.z, A, Bm);
                double[][] Om = ed.Omega;

                double[][] AT = mat3T(A);
                double[][] BT = mat3T(Bm);

                double[][] Hii = mat3Mul(mat3Mul(AT, Om), A);
                double[][] Hij = mat3Mul(mat3Mul(AT, Om), Bm);
                double[][] Hjj = mat3Mul(mat3Mul(BT, Om), Bm);
                double[][] Hji = mat3Mul(mat3Mul(BT, Om), A);

                double[] bi = mat3Vec(mat3Mul(AT, Om), e);
                double[] bj = mat3Vec(mat3Mul(BT, Om), e);

                accumulate(H, b, 3*i, 3*i, Hii);
                accumulate(H, b, 3*i, 3*j, Hij);
                accumulate(H, b, 3*j, 3*i, Hji);
                accumulate(H, b, 3*j, 3*j, Hjj);

                b[3*i] += bi[0]; b[3*i+1] += bi[1]; b[3*i+2] += bi[2];
                b[3*j] += bj[0]; b[3*j+1] += bj[1]; b[3*j+2] += bj[2];
            }

            // LM damping: H += lambda * diag(H)
            for(int k=0;k<dim;k++){
                double d = H.get(k,k);
                H.add(k,k, lambda*(d + 1e-12));
            }

            // gauge fix: anchor node 0 via huge diagonal weight
            double W = 1e12;
            H.add(0,0,W); H.add(1,1,W); H.add(2,2,W);
            b[0]=0; b[1]=0; b[2]=0;

            // solve H dx = -b
            DMatrixRMaj B = new DMatrixRMaj(dim, 1);
            for(int k=0;k<dim;k++) B.set(k, 0, -b[k]);

            LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.symmPosDef(dim);
            if(!solver.setA(H)){
                System.out.println("Solver failed to factorize.");
                break;
            }
            DMatrixRMaj dx = new DMatrixRMaj(dim, 1);
            solver.solve(B, dx);

            double step=0;
            double[] xNew = x.clone();
            for(int k=0;k<dim;k++){
                double dxi = dx.get(k,0);
                xNew[k] += dxi;
                step += dxi*dxi;
            }
            step = Math.sqrt(step);
            for(int n=0;n<N;n++) xNew[3*n+2] = wrapAngle(xNew[3*n+2]);

            double cNew = cost(xNew, N, edges);
            double cOld = cost(x, N, edges);

            if(cNew < cOld){
                x = xNew;
                lambda = Math.max(lambda/3.0, 1e-9);
                System.out.printf("iter %d: cost=%.6f step=%.3e lambda=%.2e (accepted)%n", it, cNew, step, lambda);
            } else {
                lambda = Math.min(lambda*5.0, 1e9);
                System.out.printf("iter %d: rejected, lambda=%.2e%n", it, lambda);
            }
            if(step < 1e-7) break;
        }

        System.out.println("Optimized:");
        rmse(x, gt, N);
    }
}
