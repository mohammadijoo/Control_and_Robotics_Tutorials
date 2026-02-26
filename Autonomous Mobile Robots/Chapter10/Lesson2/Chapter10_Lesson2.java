// Chapter10_Lesson2.java
// ICP Variants for Mobile Robots (2D SE(2) focus)
//
// Implements:
//   - Point-to-Point ICP (SVD via closed-form 2x2) using a simple Procrustes derivation
//   - Point-to-Line ICP (linearized least squares) using normal equations
//
// This is intentionally lightweight and uses only core Java.
// For production robotics stacks, see:
//   - ROS 2 Java bindings, or
//   - libraries such as EJML for robust linear algebra.
//
// Compile:
//   javac Chapter10_Lesson2.java
// Run:
//   java Chapter10_Lesson2
//
import java.util.*;

public class Chapter10_Lesson2 {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y){ this.x=x; this.y=y; }
        Vec2 add(Vec2 o){ return new Vec2(x+o.x, y+o.y); }
        Vec2 sub(Vec2 o){ return new Vec2(x-o.x, y-o.y); }
        Vec2 mul(double s){ return new Vec2(x*s, y*s); }
        double dot(Vec2 o){ return x*o.x + y*o.y; }
        double norm2(){ return x*x + y*y; }
        double norm(){ return Math.sqrt(norm2()); }
    }

    static class Mat2 {
        double a,b,c,d; // [[a,b],[c,d]]
        Mat2(double a,double b,double c,double d){ this.a=a; this.b=b; this.c=c; this.d=d; }
        static Mat2 rot(double th){
            double cs = Math.cos(th), sn = Math.sin(th);
            return new Mat2(cs, -sn, sn, cs);
        }
        Vec2 mul(Vec2 v){
            return new Vec2(a*v.x + b*v.y, c*v.x + d*v.y);
        }
        Mat2 mul(Mat2 M){
            return new Mat2(
                a*M.a + b*M.c, a*M.b + b*M.d,
                c*M.a + d*M.c, c*M.b + d*M.d
            );
        }
        double det(){ return a*d - b*c; }
    }

    static class ICPResult {
        Mat2 R;
        Vec2 t;
        double theta;
        ArrayList<Double> cost = new ArrayList<>();
    }

    static Vec2 mean(List<Vec2> pts){
        double sx=0, sy=0;
        for(Vec2 p: pts){ sx+=p.x; sy+=p.y; }
        int n = pts.size();
        return new Vec2(sx/n, sy/n);
    }

    static Vec2[] estimateNormals(List<Vec2> dst){
        int M = dst.size();
        Vec2[] n = new Vec2[M];
        for(int i=0;i<M;i++){
            Vec2 prev = dst.get((i-1+M)%M);
            Vec2 next = dst.get((i+1)%M);
            Vec2 tang = next.sub(prev);
            Vec2 nn = new Vec2(-tang.y, tang.x);
            double L = nn.norm();
            if(L < 1e-12) nn = new Vec2(1,0);
            else nn = nn.mul(1.0/L);
            n[i]=nn;
        }
        return n;
    }

    static int nearestBrute(Vec2 p, List<Vec2> dst, double[] bestDist){
        int best=-1;
        double bd=Double.POSITIVE_INFINITY;
        for(int j=0;j<dst.size();j++){
            Vec2 q=dst.get(j);
            double d=(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y);
            if(d<bd){ bd=d; best=j; }
        }
        bestDist[0]=Math.sqrt(bd);
        return best;
    }

    // Point-to-point: in 2D we can compute optimal rotation from cross-covariance.
    // Let H = sum (p' q'^T) = [[h11,h12],[h21,h22]].
    // The maximizing rotation satisfies:
    //   theta* = atan2(h21 - h12, h11 + h22)
    // Then t = qbar - R pbar.
    static void solveP2P(List<Vec2> P, List<Vec2> Q, Mat2[] R_out, Vec2[] t_out){
        Vec2 pbar=mean(P), qbar=mean(Q);
        double h11=0,h12=0,h21=0,h22=0;
        for(int i=0;i<P.size();i++){
            Vec2 pp=P.get(i).sub(pbar);
            Vec2 qq=Q.get(i).sub(qbar);
            h11 += pp.x*qq.x;
            h12 += pp.x*qq.y;
            h21 += pp.y*qq.x;
            h22 += pp.y*qq.y;
        }
        double theta = Math.atan2(h21 - h12, h11 + h22);
        Mat2 R = Mat2.rot(theta);
        Vec2 t = qbar.sub(R.mul(pbar));
        R_out[0]=R; t_out[0]=t;
    }

    // Solve 3x3 normal equations: (A^T A) x = A^T b
    static double[] solveSym3(double[][] M, double[] v){
        // Gaussian elimination on 3x3
        double[][] A = new double[3][4];
        for(int i=0;i<3;i++){
            System.arraycopy(M[i],0,A[i],0,3);
            A[i][3]=v[i];
        }
        for(int k=0;k<3;k++){
            int piv=k;
            for(int i=k+1;i<3;i++) if(Math.abs(A[i][k])>Math.abs(A[piv][k])) piv=i;
            double[] tmp=A[k]; A[k]=A[piv]; A[piv]=tmp;

            double pk=A[k][k];
            if(Math.abs(pk)<1e-12) return new double[]{0,0,0};
            for(int j=k;j<4;j++) A[k][j]/=pk;

            for(int i=0;i<3;i++){
                if(i==k) continue;
                double f=A[i][k];
                for(int j=k;j<4;j++) A[i][j]-=f*A[k][j];
            }
        }
        return new double[]{A[0][3],A[1][3],A[2][3]};
    }

    static void solveP2L(
        List<Vec2> P, List<Vec2> Q, List<Vec2> N,
        Mat2 R0, Vec2 t0,
        Mat2[] Rdelta_out, Vec2[] tdelta_out, double[] mean_r2_out
    ){
        // Build normal equations for delta = [dx,dy,dtheta]
        double[][] ATA = new double[3][3];
        double[] ATb = new double[3];
        double mean_r2=0;

        for(int i=0;i<P.size();i++){
            Vec2 p = R0.mul(P.get(i)).add(t0);
            Vec2 e = p.sub(Q.get(i));
            Vec2 n = N.get(i);
            double r = n.dot(e);

            double a0 = n.x;
            double a1 = n.y;
            Vec2 psrc = P.get(i);
            Vec2 perp = new Vec2(-psrc.y, psrc.x);
            double a2 = n.dot(R0.mul(perp));

            double[] a = new double[]{a0,a1,a2};
            for(int rI=0;rI<3;rI++){
                for(int cI=0;cI<3;cI++){
                    ATA[rI][cI] += a[rI]*a[cI];
                }
                ATb[rI] += a[rI]*(-r);
            }
            mean_r2 += r*r;
        }
        mean_r2 /= P.size();
        double[] delta = solveSym3(ATA, ATb);
        double dx=delta[0], dy=delta[1], dth=delta[2];
        Mat2 Rdelta = Mat2.rot(dth);
        Vec2 tdelta = new Vec2(dx,dy);
        Rdelta_out[0]=Rdelta; tdelta_out[0]=tdelta; mean_r2_out[0]=mean_r2;
    }

    static ICPResult icpSE2(List<Vec2> src, List<Vec2> dst, String variant,
                           int maxIter, double tol, double rejectDist){
        Mat2 R = Mat2.rot(0.0);
        Vec2 t = new Vec2(0.0, 0.0);

        Vec2[] normals = variant.equals("p2l") ? estimateNormals(dst) : null;

        double prevCost = Double.POSITIVE_INFINITY;
        ICPResult out = new ICPResult();

        for(int it=0; it<maxIter; it++){
            ArrayList<Vec2> P = new ArrayList<>();
            ArrayList<Vec2> Q = new ArrayList<>();
            ArrayList<Vec2> N = new ArrayList<>();

            for(int i=0;i<src.size();i++){
                Vec2 p = R.mul(src.get(i)).add(t);
                double[] bd = new double[1];
                int j = nearestBrute(p, dst, bd);
                if(j<0) continue;
                if(rejectDist>0 && bd[0] > rejectDist) continue;
                P.add(src.get(i));
                Q.add(dst.get(j));
                if(variant.equals("p2l")) N.add(normals[j]);
            }
            if(P.size()<3) break;

            double cost;
            if(variant.equals("p2p")){
                Mat2[] Rnew = new Mat2[1];
                Vec2[] tnew = new Vec2[1];
                solveP2P(P,Q,Rnew,tnew);
                R = Rnew[0];
                t = tnew[0];
                cost=0;
                for(int i=0;i<P.size();i++){
                    Vec2 e = R.mul(P.get(i)).add(t).sub(Q.get(i));
                    cost += e.norm2();
                }
                cost /= P.size();
            } else {
                Mat2[] Rdel = new Mat2[1];
                Vec2[] tdel = new Vec2[1];
                double[] meanr2 = new double[1];
                solveP2L(P,Q,N,R,t,Rdel,tdel,meanr2);
                // compose delta o current
                R = Rdel[0].mul(R);
                t = Rdel[0].mul(t).add(tdel[0]);
                cost = meanr2[0];
            }

            out.cost.add(cost);
            if(Math.abs(prevCost - cost) < tol) break;
            prevCost = cost;
        }
        out.R=R; out.t=t;
        out.theta = Math.atan2(R.c, R.a);
        return out;
    }

    public static void main(String[] args){
        // Synthetic demo
        ArrayList<Vec2> dst = new ArrayList<>();
        int M=300;
        for(int i=0;i<M;i++){
            double a = 2.0*Math.PI*(double)i/(double)M;
            double x = 2.0*Math.cos(a) + 0.3*Math.cos(5*a);
            double y = 1.0*Math.sin(a) + 0.2*Math.sin(3*a);
            dst.add(new Vec2(x,y));
        }
        double trueTheta=0.25;
        Vec2 trueT = new Vec2(0.8,-0.4);
        Mat2 Rt = Mat2.rot(trueTheta);

        ArrayList<Vec2> src = new ArrayList<>();
        for(int i=0;i<M;i++){
            Vec2 p = Rt.mul(dst.get(i)).add(trueT);
            src.add(p);
        }

        ICPResult r = icpSE2(src,dst,"p2l",60,1e-7,0.5);
        System.out.println("Estimated theta = " + r.theta);
        System.out.println("Estimated t = [" + r.t.x + ", " + r.t.y + "]");
        System.out.println("True theta = " + trueTheta);
        System.out.println("True t = [" + trueT.x + ", " + trueT.y + "]");
    }
}
