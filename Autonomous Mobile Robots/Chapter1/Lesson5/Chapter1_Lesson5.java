// Chapter1_Lesson5.java
/*
Autonomous Mobile Robots — Chapter 1, Lesson 5
Typical AMR Failure Modes (drift, slip, occlusion)

This Java program simulates the same concept as the Python/C++ versions:
  - true unicycle motion with slip (attenuation + bursts)
  - biased/noisy odometry (drift)
  - intermittent landmark measurements with occlusion/outliers
  - one-step linearized correction with gating

Compile:
  javac Chapter1_Lesson5.java
Run:
  java Chapter1_Lesson5

Output:
  trajectory_java.csv (CSV)
*/

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class Chapter1_Lesson5 {

    static double wrapToPi(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    static double[] unicycleStep(double[] x, double v, double w, double dt) {
        double px = x[0], py = x[1], th = x[2];
        double px2 = px + dt * v * Math.cos(th);
        double py2 = py + dt * v * Math.sin(th);
        double th2 = wrapToPi(th + dt * w);
        return new double[]{px2, py2, th2};
    }

    static double[] landmarkMeas(double[] x, double lx, double ly) {
        double dx = lx - x[0];
        double dy = ly - x[1];
        double r  = Math.sqrt(dx*dx + dy*dy);
        double b  = wrapToPi(Math.atan2(dy, dx) - x[2]);
        return new double[]{r, b};
    }

    static void jacobianLandmark(double[] x, double lx, double ly, double[][] H) {
        double dx = lx - x[0];
        double dy = ly - x[1];
        double q = dx*dx + dy*dy;
        if (q < 1e-12) q = 1e-12;
        double r = Math.sqrt(q);
        if (r < 1e-12) r = 1e-12;

        H[0][0] = -dx / r;
        H[0][1] = -dy / r;
        H[0][2] =  0.0;

        H[1][0] =  dy / q;
        H[1][1] = -dx / q;
        H[1][2] = -1.0;
    }

    static boolean inv2x2(double[][] A, double[][] Ainv) {
        double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
        if (Math.abs(det) < 1e-12) return false;
        double invDet = 1.0 / det;
        Ainv[0][0] =  A[1][1]*invDet;
        Ainv[0][1] = -A[0][1]*invDet;
        Ainv[1][0] = -A[1][0]*invDet;
        Ainv[1][1] =  A[0][0]*invDet;
        return true;
    }

    public static void main(String[] args) throws IOException {
        final double T = 40.0;
        final double dt = 0.02;
        final int N = (int)(T/dt);

        Random rng = new Random(7);

        // landmark
        final double lx = 8.0, ly = 6.0;

        double[] xTrue = new double[]{0.0, 0.0, 0.0};
        double[] xHat  = new double[]{0.0, 0.0, 0.0};

        // drift params
        final double bV=0.03, bW=-0.015;
        final double sV=1.03, sW=0.98;

        // slip
        final double sLongNom=0.05, sYawNom=0.03;
        final double slipBurstProb=0.02, slipBurstMag=0.35;

        // noise
        final double odoSigV=0.02, odoSigW=0.02;
        final double measSigR=0.08;
        final double measSigB=Math.toRadians(1.5);

        // correction "covariance"
        double[][] P = new double[][]{
            {0.04,0,0},
            {0,0.04,0},
            {0,0,Math.pow(Math.toRadians(5.0),2)}
        };
        final double R00 = measSigR*measSigR;
        final double R11 = measSigB*measSigB;
        final double gate = 9.21;

        // measurement schedule
        final double measPeriod = 0.5;
        double nextMeasT = 0.0;
        final double pOcclude = 0.35;
        final double pOutlier = 0.08;

        FileWriter fw = new FileWriter("trajectory_java.csv");
        fw.write("t,x_true,y_true,th_true,x_hat,y_hat,th_hat,meas_r,meas_b,meas_used\n");

        for (int k=0;k<N;k++){
            double t = k*dt;

            // command
            double vCmd = 0.6 + 0.15*Math.sin(0.4*t);
            double wCmd = 0.25*Math.sin(0.25*t) + 0.20*Math.sin(0.05*t);

            // slip
            boolean burst = rng.nextDouble() < slipBurstProb;
            double sLong = sLongNom + (burst ? slipBurstMag : 0.0);
            double sYaw  = sYawNom  + (burst ? 0.5*slipBurstMag : 0.0);
            sLong = Math.min(0.9, Math.max(0.0, sLong));
            sYaw  = Math.min(0.9, Math.max(0.0, sYaw));

            double vTrue = (1.0 - sLong)*vCmd;
            double wTrue = (1.0 - sYaw)*wCmd;
            xTrue = unicycleStep(xTrue, vTrue, wTrue, dt);

            // odometry (biased + noisy)
            double vOdo = sV*vCmd + bV + odoSigV*rng.nextGaussian();
            double wOdo = sW*wCmd + bW + odoSigW*rng.nextGaussian();
            xHat = unicycleStep(xHat, vOdo, wOdo, dt);

            double measR = Double.NaN, measB = Double.NaN;
            int measUsed = 0;

            if (t + 1e-12 >= nextMeasT){
                nextMeasT += measPeriod;
                boolean occluded = rng.nextDouble() < pOcclude;
                if (!occluded){
                    double[] z = landmarkMeas(xTrue, lx, ly);
                    z[0] += measSigR*rng.nextGaussian();
                    z[1] = wrapToPi(z[1] + measSigB*rng.nextGaussian());

                    if (rng.nextDouble() < pOutlier){
                        z[0] += 2.0*rng.nextGaussian();
                        z[1] = wrapToPi(z[1] + Math.toRadians(25.0)*rng.nextGaussian());
                    }
                    measR = z[0]; measB = z[1];

                    double[] zHat = landmarkMeas(xHat, lx, ly);
                    double r0 = z[0] - zHat[0];
                    double r1 = wrapToPi(z[1] - zHat[1]);

                    double[][] H = new double[][]{{0,0,0},{0,0,0}};
                    jacobianLandmark(xHat, lx, ly, H);

                    // S = HPH^T + R
                    double[][] HP = new double[][]{{0,0,0},{0,0,0}};
                    for (int i=0;i<2;i++){
                        for (int j=0;j<3;j++){
                            double s=0;
                            for (int m=0;m<3;m++){
                                s += H[i][m]*P[m][j];
                            }
                            HP[i][j] = s;
                        }
                    }
                    double[][] S = new double[][]{{0,0},{0,0}};
                    for (int i=0;i<2;i++){
                        for (int j=0;j<2;j++){
                            double s=0;
                            for (int m=0;m<3;m++){
                                s += HP[i][m]*H[j][m];
                            }
                            S[i][j] = s;
                        }
                    }
                    S[0][0] += R00;
                    S[1][1] += R11;

                    double[][] Sinv = new double[][]{{0,0},{0,0}};
                    if (inv2x2(S, Sinv)){
                        double d2 = r0*(Sinv[0][0]*r0 + Sinv[0][1]*r1) +
                                    r1*(Sinv[1][0]*r0 + Sinv[1][1]*r1);

                        if (d2 < gate){
                            // K = P H^T Sinv
                            double[][] PHT = new double[][]{{0,0},{0,0},{0,0}};
                            for (int i=0;i<3;i++){
                                for (int j=0;j<2;j++){
                                    double s=0;
                                    for (int m=0;m<3;m++){
                                        s += P[i][m]*H[j][m];
                                    }
                                    PHT[i][j] = s;
                                }
                            }
                            double[][] K = new double[][]{{0,0},{0,0},{0,0}};
                            for (int i=0;i<3;i++){
                                for (int j=0;j<2;j++){
                                    double s=0;
                                    for (int m=0;m<2;m++){
                                        s += PHT[i][m]*Sinv[m][j];
                                    }
                                    K[i][j] = s;
                                }
                            }

                            double dx0 = K[0][0]*r0 + K[0][1]*r1;
                            double dx1 = K[1][0]*r0 + K[1][1]*r1;
                            double dx2 = K[2][0]*r0 + K[2][1]*r1;

                            xHat[0] += dx0;
                            xHat[1] += dx1;
                            xHat[2] = wrapToPi(xHat[2] + dx2);

                            measUsed = 1;
                        }
                    }
                }
            }

            // crude inflation
            P[0][0] += 1e-4;
            P[1][1] += 1e-4;
            P[2][2] += 1e-6;

            fw.write(String.format(java.util.Locale.US,
                    "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s,%d\n",
                    t, xTrue[0], xTrue[1], xTrue[2], xHat[0], xHat[1], xHat[2],
                    Double.isNaN(measR) ? "" : String.format(java.util.Locale.US,"%.6f",measR),
                    Double.isNaN(measB) ? "" : String.format(java.util.Locale.US,"%.6f",measB),
                    measUsed));
        }

        fw.close();
        System.out.println("Wrote trajectory_java.csv");
    }
}
