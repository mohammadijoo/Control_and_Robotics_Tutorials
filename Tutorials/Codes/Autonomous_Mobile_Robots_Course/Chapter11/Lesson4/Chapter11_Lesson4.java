// Chapter11_Lesson4.java
// Chapter 11 (SLAM I) — Lesson 4: Data Association Challenges
// Minimal Java demo: chi-square gating + nearest-neighbor association (range-bearing).
//
// Dependency: EJML (ejml-simple) https://ejml.org/

import org.ejml.simple.SimpleMatrix;
import java.util.*;

public class Chapter11_Lesson4 {

    static double wrapToPi(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    // Winitzki erfinv + Wilson–Hilferty chi2 inverse (approx).
    static double erfinvApprox(double x) {
        x = Math.max(-0.999999, Math.min(0.999999, x));
        double a = 0.147;
        double ln = Math.log(1.0 - x * x);
        double t  = 2.0 / (Math.PI * a) + ln / 2.0;
        double s  = Math.sqrt(Math.sqrt(t * t - ln / a) - t);
        return (x >= 0 ? s : -s);
    }
    static double chi2invApprox(double p, int dof) {
        double z = Math.sqrt(2.0) * erfinvApprox(2.0 * p - 1.0);
        double k = (double)dof;
        return k * Math.pow(1.0 - 2.0/(9.0*k) + z*Math.sqrt(2.0/(9.0*k)), 3.0);
    }

    static double[] predictZ(double[] pose, double[] lm) {
        double dx = lm[0] - pose[0];
        double dy = lm[1] - pose[1];
        double r  = Math.hypot(dx, dy);
        double b  = wrapToPi(Math.atan2(dy, dx) - pose[2]);
        return new double[]{r, b};
    }

    static SimpleMatrix jacobianH(double[] pose, double[] lm, int j, int N) {
        double dx = lm[0] - pose[0];
        double dy = lm[1] - pose[1];
        double q  = dx*dx + dy*dy;
        double r  = Math.sqrt(q);

        int dim = 3 + 2*N;
        SimpleMatrix H = new SimpleMatrix(2, dim);

        H.set(0,0, -dx/r);  H.set(0,1, -dy/r);
        H.set(1,0,  dy/q);  H.set(1,1, -dx/q);  H.set(1,2, -1.0);

        int idx = 3 + 2*j;
        H.set(0, idx+0, dx/r);   H.set(0, idx+1, dy/r);
        H.set(1, idx+0, -dy/q);  H.set(1, idx+1, dx/q);
        return H;
    }

    static double nis(SimpleMatrix nu, SimpleMatrix S) {
        return nu.transpose().mult(S.invert()).mult(nu).get(0,0);
    }

    static int associateNN(SimpleMatrix mu, SimpleMatrix P, int N, double[] z, SimpleMatrix R, double gateProb) {
        double gamma = chi2invApprox(gateProb, 2);
        int bestJ = -1;
        double bestD2 = Double.POSITIVE_INFINITY;

        double[] pose = new double[]{mu.get(0,0), mu.get(1,0), mu.get(2,0)};

        for (int j=0;j<N;j++) {
            double[] lm = new double[]{mu.get(3+2*j,0), mu.get(3+2*j+1,0)};
            double[] zhat = predictZ(pose, lm);

            SimpleMatrix nu = new SimpleMatrix(2,1);
            nu.set(0,0, z[0] - zhat[0]);
            nu.set(1,0, wrapToPi(z[1] - zhat[1]));

            SimpleMatrix H = jacobianH(pose, lm, j, N);
            SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

            double d2 = nis(nu, S);
            if (d2 <= gamma && d2 < bestD2) { bestD2 = d2; bestJ = j; }
        }
        return bestJ;
    }

    public static void main(String[] args) {
        double[] poseTrue = {2.0, 1.0, 0.4};
        double[][] lms = {{5,2},{4,-1.5},{1,4},{7,5},{6,-2}};

        double[] poseEst = {poseTrue[0]+0.1, poseTrue[1]-0.05, poseTrue[2]+0.03};

        int N = lms.length;
        SimpleMatrix mu = new SimpleMatrix(3 + 2*N, 1);
        mu.set(0,0, poseEst[0]); mu.set(1,0, poseEst[1]); mu.set(2,0, poseEst[2]);
        for (int j=0;j<N;j++) {
            mu.set(3+2*j,0, lms[j][0]+0.2);
            mu.set(3+2*j+1,0, lms[j][1]-0.1);
        }

        // demo SPD covariance
        int dim = 3 + 2*N;
        SimpleMatrix A = SimpleMatrix.random_DDRM(dim, dim, -1, 1, new Random(7));
        SimpleMatrix P = A.mult(A.transpose());
        double maxDiag = 0;
        for (int i=0;i<dim;i++) maxDiag = Math.max(maxDiag, P.get(i,i));
        P = P.divide(maxDiag);
        double[] s = new double[dim]; s[0]=0.2; s[1]=0.2; s[2]=0.05; for(int i=3;i<dim;i++) s[i]=0.5;
        SimpleMatrix Sscale = SimpleMatrix.diag(s);
        P = Sscale.mult(P).mult(Sscale);

        double sigmaR = 0.15, sigmaB = Math.toRadians(2.0);
        SimpleMatrix R = new SimpleMatrix(2,2);
        R.set(0,0, sigmaR*sigmaR); R.set(1,1, sigmaB*sigmaB);

        double[][] Z = { predictZ(poseTrue, lms[0]),
                         predictZ(poseTrue, lms[2]),
                         predictZ(poseTrue, lms[4]) };

        for (int i=0;i<Z.length;i++) {
            int j = associateNN(mu, P, N, Z[i], R, 0.99);
            System.out.println("meas " + i + " -> landmark " + j);
        }
    }
}
