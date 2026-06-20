// Chapter10_Lesson1.java
// Point-Cloud / Scan Registration Goals — minimal educational Java example
//
// Closed-form 2D paired-point registration via SVD (Kabsch/Procrustes).
//
// Dependency (recommended): EJML (Efficient Java Matrix Library).
// Maven coordinates:
//   org.ejml:ejml-simple:0.43 (or newer compatible)
//   org.ejml:ejml-ddense:0.43
//
// This file focuses on algorithmic clarity rather than I/O.

import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

import java.util.Random;

public class Chapter10_Lesson1 {

    static SimpleMatrix rot2(double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        return new SimpleMatrix(new double[][]{
                { c, -s },
                { s,  c }
        });
    }

    static class Pose2 {
        SimpleMatrix R; // 2x2
        SimpleMatrix t; // 2x1
        Pose2(SimpleMatrix R, SimpleMatrix t) { this.R = R; this.t = t; }
    }

    static Pose2 kabsch2D(SimpleMatrix P, SimpleMatrix Q) {
        // P, Q: N x 2
        if (P.numCols() != 2 || Q.numCols() != 2 || P.numRows() != Q.numRows()) {
            throw new IllegalArgumentException("P and Q must be N x 2 and same N.");
        }

        int N = P.numRows();
        SimpleMatrix pbar = new SimpleMatrix(1,2);
        SimpleMatrix qbar = new SimpleMatrix(1,2);
        for (int i = 0; i < N; i++) {
            pbar = pbar.plus(P.extractVector(true, i));
            qbar = qbar.plus(Q.extractVector(true, i));
        }
        pbar = pbar.divide(N);
        qbar = qbar.divide(N);

        SimpleMatrix X = new SimpleMatrix(N, 2);
        SimpleMatrix Y = new SimpleMatrix(N, 2);
        for (int i = 0; i < N; i++) {
            X.insertIntoThis(i, 0, P.extractVector(true, i).minus(pbar));
            Y.insertIntoThis(i, 0, Q.extractVector(true, i).minus(qbar));
        }

        SimpleMatrix H = X.transpose().mult(Y); // 2x2
        SimpleSVD<SimpleMatrix> svd = H.svd();
        SimpleMatrix U = svd.getU();
        SimpleMatrix V = svd.getV();

        SimpleMatrix R = V.mult(U.transpose());

        // Enforce det(R)=+1
        double det = R.determinant();
        if (det < 0.0) {
            // Flip second column of V (since 2D)
            SimpleMatrix Vfix = V.copy();
            Vfix.set(0,1, -Vfix.get(0,1));
            Vfix.set(1,1, -Vfix.get(1,1));
            R = Vfix.mult(U.transpose());
        }

        // t = qbar^T - R pbar^T
        SimpleMatrix t = qbar.transpose().minus(R.mult(pbar.transpose()));

        return new Pose2(R, t);
    }

    static double rmse(SimpleMatrix A, SimpleMatrix B) {
        if (A.numRows() != B.numRows() || A.numCols() != B.numCols()) throw new IllegalArgumentException();
        int N = A.numRows();
        double s = 0.0;
        for (int i = 0; i < N; i++) {
            double dx = A.get(i,0) - B.get(i,0);
            double dy = A.get(i,1) - B.get(i,1);
            s += dx*dx + dy*dy;
        }
        return Math.sqrt(s / N);
    }

    public static void main(String[] args) {
        Random rng = new Random(7);

        int N = 200;
        SimpleMatrix P = new SimpleMatrix(N, 2);
        for (int i = 0; i < N; i++) {
            double x = -5.0 + 10.0 * rng.nextDouble();
            double y = -3.0 +  6.0 * rng.nextDouble();
            P.set(i,0, x);
            P.set(i,1, y);
        }

        double thetaGT = Math.toRadians(12.0);
        SimpleMatrix Rgt = rot2(thetaGT);
        SimpleMatrix tgt = new SimpleMatrix(new double[][]{{1.2},{-0.7}});

        double sigma = 0.02;
        SimpleMatrix Q = new SimpleMatrix(N, 2);
        for (int i = 0; i < N; i++) {
            SimpleMatrix p = P.extractVector(true, i).transpose(); // 2x1
            SimpleMatrix q = Rgt.mult(p).plus(tgt);
            q.set(0,0, q.get(0,0) + sigma * rng.nextGaussian());
            q.set(1,0, q.get(1,0) + sigma * rng.nextGaussian());
            Q.set(i,0, q.get(0,0));
            Q.set(i,1, q.get(1,0));
        }

        Pose2 est = kabsch2D(P, Q);

        double thetaHat = Math.atan2(est.R.get(1,0), est.R.get(0,0));

        System.out.printf("thetaGT(deg)=%.6f, tGT=[%.4f, %.4f]%n",
                Math.toDegrees(thetaGT), tgt.get(0), tgt.get(1));
        System.out.printf("thetaHat(deg)=%.6f, tHat=[%.4f, %.4f]%n",
                Math.toDegrees(thetaHat), est.t.get(0), est.t.get(1));

        // Apply transform to P
        SimpleMatrix Qhat = new SimpleMatrix(N, 2);
        for (int i = 0; i < N; i++) {
            SimpleMatrix p = P.extractVector(true, i).transpose();
            SimpleMatrix q = est.R.mult(p).plus(est.t);
            Qhat.set(i,0, q.get(0));
            Qhat.set(i,1, q.get(1));
        }

        System.out.printf("RMSE(Qhat,Q)=%.6f%n", rmse(Qhat, Q));
    }
}
