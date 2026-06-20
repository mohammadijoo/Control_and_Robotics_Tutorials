// Chapter9_Lesson3.java
// Feature-Based Mapping (Landmarks) with per-landmark EKF (pose assumed known).
// This version avoids external matrix libraries by implementing 2x2 algebra manually.
// For real robotics stacks, typical Java choices include EJML or Apache Commons Math.

import java.util.*;
import static java.lang.Math.*;

public class Chapter9_Lesson3 {

    static class Pose2 { double x,y,th; Pose2(double x,double y,double th){this.x=x;this.y=y;this.th=th;} }
    static class MeasRB { double r,b; MeasRB(double r,double b){this.r=r;this.b=b;} }

    static double wrap(double a){
        while(a>PI) a-=2*PI;
        while(a<-PI) a+=2*PI;
        return a;
    }

    static MeasRB h(Pose2 x, double[] m){
        double dx=m[0]-x.x, dy=m[1]-x.y;
        double r=sqrt(dx*dx+dy*dy);
        double b=wrap(atan2(dy,dx)-x.th);
        return new MeasRB(r,b);
    }

    static double[][] H(Pose2 x, double[] m){
        double dx=m[0]-x.x, dy=m[1]-x.y;
        double q=dx*dx+dy*dy;
        double r=sqrt(q);
        if(r<1e-9){ r=1e-9; q=r*r; }
        return new double[][]{
                {dx/r, dy/r},
                {-dy/q, dx/q}
        };
    }

    static double[] invh(Pose2 x, MeasRB z){
        double ang = x.th + z.b;
        return new double[]{ x.x + z.r*cos(ang), x.y + z.r*sin(ang) };
    }

    // 2x2 helpers
    static double det2(double[][] A){ return A[0][0]*A[1][1]-A[0][1]*A[1][0]; }
    static double[][] inv2(double[][] A){
        double d=det2(A);
        return new double[][]{{ A[1][1]/d, -A[0][1]/d },{ -A[1][0]/d, A[0][0]/d }};
    }
    static double[][] add2(double[][] A,double[][] B){
        return new double[][]{{A[0][0]+B[0][0],A[0][1]+B[0][1]},{A[1][0]+B[1][0],A[1][1]+B[1][1]}};
    }
    static double[][] sub2(double[][] A,double[][] B){
        return new double[][]{{A[0][0]-B[0][0],A[0][1]-B[0][1]},{A[1][0]-B[1][0],A[1][1]-B[1][1]}};
    }
    static double[][] mul2(double[][] A,double[][] B){
        return new double[][]{
                {A[0][0]*B[0][0]+A[0][1]*B[1][0], A[0][0]*B[0][1]+A[0][1]*B[1][1]},
                {A[1][0]*B[0][0]+A[1][1]*B[1][0], A[1][0]*B[0][1]+A[1][1]*B[1][1]}
        };
    }
    static double[] mul2(double[][] A,double[] v){
        return new double[]{ A[0][0]*v[0]+A[0][1]*v[1], A[1][0]*v[0]+A[1][1]*v[1] };
    }
    static double[][] trans2(double[][] A){
        return new double[][]{{A[0][0],A[1][0]},{A[0][1],A[1][1]}};
    }

    static class Landmark {
        double[] mu = new double[2];
        double[][] P = new double[][]{{1,0},{0,1}};
        boolean active=true;
    }

    static class Mapper {
        double[][] R; double gate;
        ArrayList<Landmark> L = new ArrayList<>();
        Mapper(double[][] R, double gate){ this.R=R; this.gate=gate; }

        void add(Pose2 x, MeasRB z){
            Landmark lm = new Landmark();
            lm.mu = invh(x,z);
            lm.P = new double[][]{{2.25,0},{0,2.25}}; // 1.5^2
            L.add(lm);
        }

        int associate(Pose2 x, MeasRB z, double[] bestD2){
            if(L.isEmpty()){ bestD2[0]=Double.POSITIVE_INFINITY; return -1; }
            int best=-1; double bestd=Double.POSITIVE_INFINITY;
            for(int j=0;j<L.size();j++){
                Landmark lm=L.get(j); if(!lm.active) continue;
                double[][] Hj=H(x,lm.mu);
                MeasRB zhat=h(x,lm.mu);
                double[] y=new double[]{ z.r-zhat.r, wrap(z.b-zhat.b) };
                double[][] S=add2(mul2(mul2(Hj,lm.P),trans2(Hj)), R);
                double[][] Sinv=inv2(S);
                double d2 = y[0]*(Sinv[0][0]*y[0]+Sinv[0][1]*y[1]) + y[1]*(Sinv[1][0]*y[0]+Sinv[1][1]*y[1]);
                if(d2<bestd){ bestd=d2; best=j; }
            }
            bestD2[0]=bestd;
            if(bestd<=gate) return best;
            return -1;
        }

        void update(Pose2 x, MeasRB z, int j){
            Landmark lm=L.get(j);
            double[][] Hj=H(x,lm.mu);
            MeasRB zhat=h(x,lm.mu);
            double[] y=new double[]{ z.r-zhat.r, wrap(z.b-zhat.b) };

            double[][] S=add2(mul2(mul2(Hj,lm.P),trans2(Hj)), R);
            double[][] Sinv=inv2(S);
            // K = P H^T S^{-1}; size 2x2
            double[][] K = mul2(mul2(lm.P, trans2(Hj)), Sinv);

            double[] Ky = mul2(K, y);
            lm.mu[0] += Ky[0];
            lm.mu[1] += Ky[1];

            double[][] I = new double[][]{{1,0},{0,1}};
            lm.P = mul2(sub2(I, mul2(K,Hj)), lm.P);

            // symmetrize (manual)
            double a = 0.5*(lm.P[0][1] + lm.P[1][0]);
            lm.P[0][1]=a; lm.P[1][0]=a;
        }
    }

    public static void main(String[] args){
        Random rng = new Random(7);
        // true landmarks
        double[][] M = new double[][]{
                {-6,5}, {-2,-4}, {4,6}, {7,-2}, {1,1.5}, {-7,-6}
        };

        int T=180;
        ArrayList<Pose2> X = new ArrayList<>();
        for(int k=0;k<T;k++){
            double t = 2*PI*(double)k/(double)(T-1);
            double px = 2.5*cos(t) + 0.5*cos(3*t);
            double py = 2.0*sin(t);
            double t2 = 2*PI*(double)Math.min(k+1,T-1)/(double)(T-1);
            double px2 = 2.5*cos(t2) + 0.5*cos(3*t2);
            double py2 = 2.0*sin(t2);
            double th = wrap(atan2(py2-py, px2-px));
            X.add(new Pose2(px,py,th));
        }

        double rMax=9.0;
        double sigmaR=0.15;
        double sigmaB=2.0*PI/180.0;
        double[][] R = new double[][]{{sigmaR*sigmaR,0},{0,sigmaB*sigmaB}};

        Mapper mapper = new Mapper(R, 9.21);

        for(int k=0;k<T;k++){
            Pose2 x = X.get(k);
            for(int i=0;i<M.length;i++){
                MeasRB z = h(x, M[i]);
                if(z.r <= rMax){
                    // add noise
                    z.r += sigmaR * rng.nextGaussian();
                    z.b = wrap(z.b + sigmaB * rng.nextGaussian());

                    double[] bestD2=new double[1];
                    int j = mapper.associate(x, z, bestD2);
                    if(j<0) mapper.add(x, z);
                    else mapper.update(x, z, j);
                }
            }
        }

        System.out.println("Estimated landmarks (mu):");
        for(int j=0;j<mapper.L.size();j++){
            Landmark lm = mapper.L.get(j);
            System.out.printf("%d: %.4f %.4f%n", j, lm.mu[0], lm.mu[1]);
        }
    }
}
