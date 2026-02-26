// Chapter1_Lesson4.java
// Sensing–Estimation–Navigation pipeline demo (2D unicycle) using prior + WLS correction.
// Build:
//   javac Chapter1_Lesson4.java
// Run:
//   java Chapter1_Lesson4
//
// Output: "traj_ch1_l4_java.csv" with true and estimated pose.
// Robotics ecosystem pointers: ROSJava (rosjava), EJML for matrices, JVX? (not required here).
// This demo uses small hand-coded linear algebra to stay self-contained.

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class Chapter1_Lesson4 {

    static double wrapAngle(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0.0) a += twoPi;
        return a - Math.PI;
    }

    static double[] unicycleStep(double[] x, double v, double w, double dt) {
        double[] xn = x.clone();
        double th = x[2];
        xn[0] = x[0] + v * dt * Math.cos(th);
        xn[1] = x[1] + v * dt * Math.sin(th);
        xn[2] = wrapAngle(x[2] + w * dt);
        return xn;
    }

    static double[] measModel(double[] x, double[] lm) {
        double dx = lm[0] - x[0];
        double dy = lm[1] - x[1];
        double r = Math.hypot(dx, dy);
        double b = wrapAngle(Math.atan2(dy, dx) - x[2]);
        return new double[]{r, b};
    }

    static double[][] measJacobian(double[] x, double[] lm) {
        double dx = lm[0] - x[0];
        double dy = lm[1] - x[1];
        double q = dx*dx + dy*dy;
        q = Math.max(q, 1e-12);
        double r = Math.sqrt(q);
        r = Math.max(r, 1e-12);

        double[][] H = new double[2][3];
        // range
        H[0][0] = -dx / r;
        H[0][1] = -dy / r;
        H[0][2] = 0.0;
        // bearing
        H[1][0] =  dy / q;
        H[1][1] = -dx / q;
        H[1][2] = -1.0;
        return H;
    }

    // Small 3x3 linear algebra helpers
    static double[][] matAdd3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) C[i][j] = A[i][j] + B[i][j];
        return C;
    }
    static double[][] matMul3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
            double s=0.0;
            for (int k=0;k<3;k++) s += A[i][k]*B[k][j];
            C[i][j]=s;
        }
        return C;
    }
    static double[] matVec3(double[][] A, double[] x) {
        double[] y = new double[3];
        for (int i=0;i<3;i++) {
            y[i]=A[i][0]*x[0] + A[i][1]*x[1] + A[i][2]*x[2];
        }
        return y;
    }
    static double[][] transpose3(double[][] A) {
        double[][] T = new double[3][3];
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) T[i][j] = A[j][i];
        return T;
    }

    // Inverse of 3x3 matrix via adjugate (sufficient for this small demo)
    static double[][] inv3(double[][] A) {
        double a=A[0][0], b=A[0][1], c=A[0][2];
        double d=A[1][0], e=A[1][1], f=A[1][2];
        double g=A[2][0], h=A[2][1], i=A[2][2];
        double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        if (Math.abs(det) < 1e-12) det = (det >= 0 ? 1e-12 : -1e-12);

        double[][] inv = new double[3][3];
        inv[0][0] =  (e*i - f*h)/det;
        inv[0][1] = -(b*i - c*h)/det;
        inv[0][2] =  (b*f - c*e)/det;
        inv[1][0] = -(d*i - f*g)/det;
        inv[1][1] =  (a*i - c*g)/det;
        inv[1][2] = -(a*f - c*d)/det;
        inv[2][0] =  (d*h - e*g)/det;
        inv[2][1] = -(a*h - b*g)/det;
        inv[2][2] =  (a*e - b*d)/det;
        return inv;
    }

    // Solve 3x3 system A x = b (Gaussian elimination)
    static double[] solve3(double[][] A, double[] b) {
        double[][] M = new double[3][4];
        for (int r=0;r<3;r++) {
            System.arraycopy(A[r], 0, M[r], 0, 3);
            M[r][3] = b[r];
        }
        for (int col=0; col<3; col++) {
            // pivot
            int piv = col;
            for (int r=col+1;r<3;r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
            double[] tmp = M[col]; M[col] = M[piv]; M[piv] = tmp;

            double pivot = M[col][col];
            if (Math.abs(pivot) < 1e-12) pivot = (pivot>=0 ? 1e-12 : -1e-12);
            for (int c=col;c<4;c++) M[col][c] /= pivot;

            for (int r=0;r<3;r++) if (r != col) {
                double factor = M[r][col];
                for (int c=col;c<4;c++) M[r][c] -= factor * M[col][c];
            }
        }
        return new double[]{M[0][3], M[1][3], M[2][3]};
    }

    static void wlsUpdateIterated(
            double[] xPrior, double[][] PPrior,
            double[][] zList, double[][] lmList, double[][] R, int iters,
            double[] xPost, double[][] PPost) {

        int m = zList.length;
        if (m == 0) {
            System.arraycopy(xPrior, 0, xPost, 0, 3);
            for (int r=0;r<3;r++) System.arraycopy(PPrior[r], 0, PPost[r], 0, 3);
            return;
        }

        // Build R^{-1} for each measurement (2x2)
        double detR = R[0][0]*R[1][1] - R[0][1]*R[1][0];
        double[][] Rinv = new double[][] {
                { R[1][1]/detR, -R[0][1]/detR},
                {-R[1][0]/detR,  R[0][0]/detR}
        };

        double[][] Pinv = inv3(PPrior);
        double[] x = xPrior.clone();

        for (int it=0; it<iters; it++) {
            // Accumulate normal equation pieces:
            // A = Pinv + sum H^T Rinv H,  b = sum H^T Rinv res + Pinv(xPrior - x)
            double[][] A = new double[3][3];
            for (int r=0;r<3;r++) System.arraycopy(Pinv[r], 0, A[r], 0, 3);

            double[] b = matVec3(Pinv, new double[]{xPrior[0]-x[0], xPrior[1]-x[1], wrapAngle(xPrior[2]-x[2])});

            for (int j=0; j<m; j++) {
                double[] z = zList[j];
                double[] lm = lmList[j];

                double[] h = measModel(x, lm);
                double[] res = new double[]{z[0]-h[0], wrapAngle(z[1]-h[1])};

                double[][] H = measJacobian(x, lm); // 2x3

                // Compute H^T Rinv H (3x3) and H^T Rinv res (3)
                double[][] RtH = new double[2][3];
                for (int r=0;r<2;r++) for (int c=0;c<3;c++) {
                    RtH[r][c] = Rinv[r][0]*H[0][c] + Rinv[r][1]*H[1][c];
                }
                // add to A: H^T * (Rinv*H)
                for (int r=0;r<3;r++) for (int c=0;c<3;c++) {
                    A[r][c] += H[0][r]*RtH[0][c] + H[1][r]*RtH[1][c];
                }

                // add to b: H^T * (Rinv*res)
                double r0 = Rinv[0][0]*res[0] + Rinv[0][1]*res[1];
                double r1 = Rinv[1][0]*res[0] + Rinv[1][1]*res[1];
                b[0] += H[0][0]*r0 + H[1][0]*r1;
                b[1] += H[0][1]*r0 + H[1][1]*r1;
                b[2] += H[0][2]*r0 + H[1][2]*r1;
            }

            double[] delta = solve3(A, b);
            x[0] += delta[0];
            x[1] += delta[1];
            x[2] = wrapAngle(x[2] + delta[2]);
        }

        // Approximate posterior covariance by recomputing A at final x
        double[][] A = new double[3][3];
        for (int r=0;r<3;r++) System.arraycopy(Pinv[r], 0, A[r], 0, 3);
        for (int j=0; j<m; j++) {
            double[][] H = measJacobian(x, lmList[j]);
            // A += H^T Rinv H
            double[][] RtH = new double[2][3];
            for (int r=0;r<2;r++) for (int c=0;c<3;c++) {
                RtH[r][c] = Rinv[r][0]*H[0][c] + Rinv[r][1]*H[1][c];
            }
            for (int r=0;r<3;r++) for (int c=0;c<3;c++) {
                A[r][c] += H[0][r]*RtH[0][c] + H[1][r]*RtH[1][c];
            }
        }
        double[][] P = inv3(A);

        System.arraycopy(x, 0, xPost, 0, 3);
        for (int r=0;r<3;r++) System.arraycopy(P[r], 0, PPost[r], 0, 3);
    }

    public static void main(String[] args) throws IOException {
        Random rng = new Random(7);

        double[][] landmarks = new double[][] {
                {5.0, 0.0},
                {6.0, 6.0},
                {0.0, 6.0}
        };

        double dt = 0.1;
        int N = 350;

        double[] xTrue = new double[]{0.0, 0.0, 0.0};
        double[] xHat  = new double[]{0.0, 0.0, 0.0};

        double[][] P = new double[][] {
                {0.15*0.15, 0.0, 0.0},
                {0.0, 0.15*0.15, 0.0},
                {0.0, 0.0, Math.pow(Math.toRadians(8.0), 2)}
        };

        double sigmaV = 0.08;
        double sigmaW = Math.toRadians(3.0);
        double sigmaR = 0.12;
        double sigmaB = Math.toRadians(2.0);

        double[] goal = new double[]{7.0, 7.0};
        double kHeading = 1.6;
        double vMax = 0.9;

        double[][] R = new double[][] {
                {sigmaR*sigmaR, 0.0},
                {0.0, sigmaB*sigmaB}
        };

        FileWriter csv = new FileWriter("traj_ch1_l4_java.csv");
        csv.write("k,x_true,y_true,th_true,x_hat,y_hat,th_hat\n");

        for (int k=0; k<N; k++) {
            // Navigation uses estimate
            double dx = goal[0] - xHat[0];
            double dy = goal[1] - xHat[1];
            double dist = Math.hypot(dx, dy);
            double desired = Math.atan2(dy, dx);
            double headingErr = wrapAngle(desired - xHat[2]);

            double vCmd = vMax * Math.tanh(dist);
            double wCmd = kHeading * headingErr;

            // True motion includes mild slip
            double slip = Math.toRadians(0.35) * rng.nextGaussian();
            xTrue = unicycleStep(xTrue, vCmd, wCmd + slip/dt, dt);

            // Odometry
            double vMeas = vCmd + sigmaV * rng.nextGaussian();
            double wMeas = wCmd + sigmaW * rng.nextGaussian();

            double[] xPrior = unicycleStep(xHat, vMeas, wMeas, dt);

            double th = xHat[2];
            double[][] F = new double[][] {
                    {1.0, 0.0, -vMeas*dt*Math.sin(th)},
                    {0.0, 1.0,  vMeas*dt*Math.cos(th)},
                    {0.0, 0.0,  1.0}
            };

            double[][] Q = new double[][] {
                    {Math.pow(sigmaV*dt,2), 0.0, 0.0},
                    {0.0, Math.pow(sigmaV*dt,2), 0.0},
                    {0.0, 0.0, Math.pow(sigmaW*dt,2)}
            };

            double[][] FP = matMul3(F, P);
            double[][] FPFT = matMul3(FP, transpose3(F));
            double[][] PPrior = matAdd3(FPFT, Q);

            // Measurements
            double maxRange = 8.0;
            int count = 0;
            double[][] zListTemp = new double[landmarks.length][2];
            double[][] lmListTemp = new double[landmarks.length][2];
            for (int j=0; j<landmarks.length; j++) {
                double[] zTrue = measModel(xTrue, landmarks[j]);
                if (zTrue[0] <= maxRange) {
                    double[] zNoisy = new double[] {
                            zTrue[0] + sigmaR * rng.nextGaussian(),
                            wrapAngle(zTrue[1] + sigmaB * rng.nextGaussian())
                    };
                    zListTemp[count] = zNoisy;
                    lmListTemp[count] = landmarks[j];
                    count++;
                }
            }
            double[][] zList = new double[count][2];
            double[][] lmList = new double[count][2];
            for (int j=0; j<count; j++) {
                zList[j] = zListTemp[j];
                lmList[j] = lmListTemp[j];
            }

            double[] xPost = new double[3];
            double[][] PPost = new double[3][3];
            wlsUpdateIterated(xPrior, PPrior, zList, lmList, R, 2, xPost, PPost);

            xHat = xPost;
            P = PPost;

            csv.write(String.format("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    k, xTrue[0], xTrue[1], xTrue[2], xHat[0], xHat[1], xHat[2]));
        }

        csv.close();
        System.out.println("Wrote traj_ch1_l4_java.csv");
    }
}
