// Chapter12_Lesson4.java
// Graph-Based SLAM (2D pose graph) with Huber IRLS (numeric Jacobians) in pure Java.
// Educational: focuses on the robust weighting logic rather than performance.

import java.util.ArrayList;
import java.util.List;

public class Chapter12_Lesson4 {

    static class Edge {
        int i, j;
        double[] z;      // [dx, dy, dtheta] = pose_i^{-1} ∘ pose_j in i-frame
        double[][] Omega; // 3x3 information matrix
        Edge(int i, int j, double[] z, double[][] Omega) {
            this.i=i; this.j=j; this.z=z; this.Omega=Omega;
        }
    }

    static double wrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    static double[] between(double[] xi, double[] xj) {
        double dx = xj[0] - xi[0];
        double dy = xj[1] - xi[1];
        double c = Math.cos(xi[2]), s = Math.sin(xi[2]);
        double xr =  c * dx + s * dy;
        double yr = -s * dx + c * dy;
        double tr = wrap(xj[2] - xi[2]);
        return new double[]{xr, yr, tr};
    }

    static double huberWeightFromS(double s, double delta) {
        double r = Math.sqrt(Math.max(s, 1e-12));
        return (r <= delta) ? 1.0 : (delta / r);
    }

    static class EdgeLinearization {
        double[] e;     // 3
        double[][] Ji;  // 3x3
        double[][] Jj;  // 3x3
        EdgeLinearization(double[] e, double[][] Ji, double[][] Jj) {
            this.e=e; this.Ji=Ji; this.Jj=Jj;
        }
    }

    static EdgeLinearization numericJacobianEdge(double[] xi, double[] xj, double[] z, double eps) {
        double[] e0 = err(xi, xj, z);
        double[][] Ji = new double[3][3];
        double[][] Jj = new double[3][3];

        for (int k=0; k<3; k++) {
            double[] d = new double[]{0,0,0};
            d[k] = eps;

            double[] xi2 = new double[]{xi[0]+d[0], xi[1]+d[1], xi[2]+d[2]};
            double[] ei = err(xi2, xj, z);
            for (int r=0; r<3; r++) Ji[r][k] = (ei[r]-e0[r]) / eps;

            double[] xj2 = new double[]{xj[0]+d[0], xj[1]+d[1], xj[2]+d[2]};
            double[] ej = err(xi, xj2, z);
            for (int r=0; r<3; r++) Jj[r][k] = (ej[r]-e0[r]) / eps;
        }
        return new EdgeLinearization(e0, Ji, Jj);
    }

    static double[] err(double[] xi, double[] xj, double[] z) {
        double[] zhat = between(xi, xj);
        double[] e = new double[]{zhat[0]-z[0], zhat[1]-z[1], wrap(zhat[2]-z[2])};
        return e;
    }

    static double quadForm(double[] v, double[][] A) {
        double[] Av = new double[3];
        for (int i=0;i<3;i++){
            Av[i]=0;
            for(int j=0;j<3;j++) Av[i]+=A[i][j]*v[j];
        }
        double s=0;
        for(int i=0;i<3;i++) s+=v[i]*Av[i];
        return s;
    }

    static double[][] matMulT(double[][] J, double[][] A, double[][] K) {
        // return J^T A K ; here all are 3x3
        double[][] out = new double[3][3];
        for (int r=0;r<3;r++){
            for (int c=0;c<3;c++){
                double sum=0;
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        sum += J[i][r] * A[i][j] * K[j][c];
                    }
                }
                out[r][c]=sum;
            }
        }
        return out;
    }

    static double[] matVecMulT(double[][] J, double[][] A, double[] e) {
        // return J^T A e
        double[] out = new double[3];
        for (int r=0;r<3;r++){
            double sum=0;
            for (int i=0;i<3;i++){
                for (int j=0;j<3;j++){
                    sum += J[i][r] * A[i][j] * e[j];
                }
            }
            out[r]=sum;
        }
        return out;
    }

    static void add3(double[][] H, int ra, int ca, double[][] blk) {
        for (int r=0;r<3;r++){
            for(int c=0;c<3;c++){
                H[ra+r][ca+c]+=blk[r][c];
            }
        }
    }
    static void add3v(double[] b, int ra, double[] v) {
        for(int r=0;r<3;r++) b[ra+r]+=v[r];
    }

    static double[] solveLinear(double[][] A, double[] b) {
        // naive Gaussian elimination (for small systems)
        int n = b.length;
        double[][] M = new double[n][n];
        double[] x = new double[n];
        double[] bb = new double[n];
        for(int i=0;i<n;i++){
            System.arraycopy(A[i], 0, M[i], 0, n);
            bb[i]=b[i];
        }
        // forward
        for(int k=0;k<n;k++){
            // pivot
            int piv=k;
            double best=Math.abs(M[k][k]);
            for(int i=k+1;i<n;i++){
                if(Math.abs(M[i][k])>best){best=Math.abs(M[i][k]);piv=i;}
            }
            if(piv!=k){
                double[] tmp=M[k]; M[k]=M[piv]; M[piv]=tmp;
                double t=bb[k]; bb[k]=bb[piv]; bb[piv]=t;
            }
            double diag=M[k][k];
            if(Math.abs(diag)<1e-12) continue;
            for(int i=k+1;i<n;i++){
                double f=M[i][k]/diag;
                for(int j=k;j<n;j++) M[i][j]-=f*M[k][j];
                bb[i]-=f*bb[k];
            }
        }
        // back substitution
        for(int i=n-1;i>=0;i--){
            double sum=bb[i];
            for(int j=i+1;j<n;j++) sum-=M[i][j]*x[j];
            double diag=M[i][i];
            x[i]=(Math.abs(diag)<1e-12)?0:(sum/diag);
        }
        return x;
    }

    static double[][] diagOmega(double sxy, double sth) {
        double[][] O = new double[3][3];
        O[0][0] = 1.0/(sxy*sxy);
        O[1][1] = 1.0/(sxy*sxy);
        O[2][2] = 1.0/(sth*sth);
        return O;
    }

    static double[][] scaleMat(double a, double[][] M) {
        double[][] out = new double[3][3];
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) out[i][j]=a*M[i][j];
        return out;
    }

    public static void main(String[] args) {
        int N = 20;
        double[][] x = new double[N][3];

        // initial chain
        for (int k=1;k<N;k++){
            x[k][0] = x[k-1][0] + 0.3;
            x[k][1] = 0.0;
            x[k][2] = wrap(x[k-1][2] + 0.05);
        }

        double sigmaXY=0.05, sigmaTH=0.02;
        double[][] Omega = diagOmega(sigmaXY, sigmaTH);

        List<Edge> edges = new ArrayList<>();
        for(int k=0;k<N-1;k++){
            edges.add(new Edge(k, k+1, new double[]{0.3,0.0,0.05}, Omega));
        }
        // bad loop closure (outlier)
        edges.add(new Edge(0, N-1, new double[]{2.0,-1.0,1.0}, Omega));

        // Optimize with IRLS-Huber
        int iters=15;
        double delta=1.5;
        double eps=1e-6;
        double damping=1e-6;

        // Fix node 0 (remove gauge): variables are nodes 1..N-1
        int varN = N-1;
        int dim = 3*varN;

        for(int it=0; it<iters; it++){
            double[][] H = new double[dim][dim];
            double[] b = new double[dim];

            double sumS=0, sumRho=0;

            for(Edge ed : edges){
                int i=ed.i, j=ed.j;
                EdgeLinearization lin = numericJacobianEdge(x[i], x[j], ed.z, eps);
                double s = quadForm(lin.e, ed.Omega);
                sumS += s;

                double w = huberWeightFromS(s, delta);
                double r = Math.sqrt(Math.max(s, 1e-12));
                double rho = (r <= delta) ? s : (2.0*delta*r - delta*delta);
                sumRho += rho;

                double[][] W = scaleMat(w, ed.Omega);

                // blocks if variable (skip node 0)
                Integer ci = (i==0)? null : 3*(i-1);
                Integer cj = (j==0)? null : 3*(j-1);

                // Hii, Hij, gj
                if(ci != null){
                    double[][] Hii = matMulT(lin.Ji, W, lin.Ji);
                    double[] gi = matVecMulT(lin.Ji, W, lin.e);
                    add3(H, ci, ci, Hii);
                    add3v(b, ci, gi);
                }
                if(cj != null){
                    double[][] Hjj = matMulT(lin.Jj, W, lin.Jj);
                    double[] gj = matVecMulT(lin.Jj, W, lin.e);
                    add3(H, cj, cj, Hjj);
                    add3v(b, cj, gj);
                }
                if(ci != null && cj != null){
                    double[][] Hij = matMulT(lin.Ji, W, lin.Jj);
                    double[][] Hji = matMulT(lin.Jj, W, lin.Ji);
                    add3(H, ci, cj, Hij);
                    add3(H, cj, ci, Hji);
                }
            }

            // damping
            for(int d=0; d<dim; d++) H[d][d] += damping;

            // solve H dx = -b
            double[] rhs = new double[dim];
            for(int k=0;k<dim;k++) rhs[k] = -b[k];
            double[] dx = solveLinear(H, rhs);

            double norm=0;
            for(double v: dx) norm += v*v;
            norm = Math.sqrt(norm);

            // apply update
            for(int node=1; node<N; node++){
                int c = 3*(node-1);
                x[node][0] += dx[c+0];
                x[node][1] += dx[c+1];
                x[node][2] = wrap(x[node][2] + dx[c+2]);
            }

            System.out.printf("iter %02d  sumS=%.3f  sumRho=%.3f  |dx|=%.3e%n", it, sumS, sumRho, norm);
            if(norm < 1e-8) break;
        }

        System.out.println("Final pose of last node: x=" + x[N-1][0] + " y=" + x[N-1][1] + " th=" + x[N-1][2]);
    }
}
