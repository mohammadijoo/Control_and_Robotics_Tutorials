/*
Chapter16_Lesson1.java
Sensing Dynamic Obstacles — Minimal Kalman Filter (Java, teaching baseline)

This is a compact 2D constant-velocity KF for a single track.
For production robotics, consider:
- ROS2 Java bindings, or
- EJML for linear algebra.

Here we implement small matrix ops manually to avoid external deps.
*/

import java.util.Random;

public class Chapter16_Lesson1 {

    // 4x4 matrix multiply: C = A*B
    static double[][] mul44(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i=0;i<4;i++){
            for (int j=0;j<4;j++){
                double s=0;
                for (int k=0;k<4;k++) s += A[i][k]*B[k][j];
                C[i][j]=s;
            }
        }
        return C;
    }

    // 4x4 * 4x1
    static double[] mul4v(double[][] A, double[] x) {
        double[] y = new double[4];
        for(int i=0;i<4;i++){
            double s=0;
            for(int k=0;k<4;k++) s += A[i][k]*x[k];
            y[i]=s;
        }
        return y;
    }

    // transpose 4x4
    static double[][] T44(double[][] A){
        double[][] B=new double[4][4];
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) B[i][j]=A[j][i];
        return B;
    }

    // 2x4 * 4x4 * 4x2 => 2x2 helper pieces
    static double[][] HPHt(double[][] H, double[][] P){
        // tmp = H*P => 2x4
        double[][] tmp = new double[2][4];
        for(int i=0;i<2;i++){
            for(int j=0;j<4;j++){
                double s=0;
                for(int k=0;k<4;k++) s += H[i][k]*P[k][j];
                tmp[i][j]=s;
            }
        }
        // S = tmp*H^T => 2x2
        double[][] S = new double[2][2];
        for(int i=0;i<2;i++){
            for(int j=0;j<2;j++){
                double s=0;
                for(int k=0;k<4;k++) s += tmp[i][k]*H[j][k]; // H^T
                S[i][j]=s;
            }
        }
        return S;
    }

    // invert 2x2
    static double[][] inv2(double[][] A){
        double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
        double[][] B = new double[2][2];
        B[0][0]= A[1][1]/det;
        B[1][1]= A[0][0]/det;
        B[0][1]= -A[0][1]/det;
        B[1][0]= -A[1][0]/det;
        return B;
    }

    // 4x4 + 4x4
    static double[][] add44(double[][] A, double[][] B){
        double[][] C=new double[4][4];
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) C[i][j]=A[i][j]+B[i][j];
        return C;
    }

    public static void main(String[] args){
        double dt = 0.1;
        double sigmaA = 0.7;
        double sigmaZ = 0.35;

        double[][] F = {
            {1,0,dt,0},
            {0,1,0,dt},
            {0,0,1,0},
            {0,0,0,1}
        };

        double q = sigmaA*sigmaA;
        double[][] Q = {
            {q*Math.pow(dt,4)/4.0, 0, q*Math.pow(dt,3)/2.0, 0},
            {0, q*Math.pow(dt,4)/4.0, 0, q*Math.pow(dt,3)/2.0},
            {q*Math.pow(dt,3)/2.0, 0, q*Math.pow(dt,2), 0},
            {0, q*Math.pow(dt,3)/2.0, 0, q*Math.pow(dt,2)}
        };

        double[][] H = {
            {1,0,0,0},
            {0,1,0,0}
        };

        double[][] R = {
            {sigmaZ*sigmaZ, 0},
            {0, sigmaZ*sigmaZ}
        };

        double[] x = {0,0,0,0};
        double[][] P = {
            {1,0,0,0},
            {0,1,0,0},
            {0,0,2,0},
            {0,0,0,2}
        };

        // truth
        double[] xt = {0,0,1.0,0.6};
        Random rng = new Random(0);

        for(int k=0;k<60;k++){
            // propagate truth
            xt[0] += dt*xt[2];
            xt[1] += dt*xt[3];

            // measurement
            double[] z = {
                xt[0] + sigmaZ*rng.nextGaussian(),
                xt[1] + sigmaZ*rng.nextGaussian()
            };

            // predict
            x = mul4v(F, x);
            P = add44(mul44(mul44(F,P), T44(F)), Q);

            // innovation y = z - Hx
            double[] Hx = { x[0], x[1] };
            double[] y = { z[0]-Hx[0], z[1]-Hx[1] };

            // S = HPH^T + R
            double[][] S = HPHt(H,P);
            S[0][0] += R[0][0]; S[1][1] += R[1][1];

            double[][] S_inv = inv2(S);

            // K = P H^T S^{-1}  => (4x4)(4x2)(2x2) = 4x2
            double[][] PHt = new double[4][2];
            for(int i=0;i<4;i++){
                PHt[i][0]=P[i][0];
                PHt[i][1]=P[i][1];
            }
            double[][] K = new double[4][2];
            for(int i=0;i<4;i++){
                for(int j=0;j<2;j++){
                    K[i][j]=PHt[i][0]*S_inv[0][j] + PHt[i][1]*S_inv[1][j];
                }
            }

            // x = x + K y
            for(int i=0;i<4;i++){
                x[i] = x[i] + K[i][0]*y[0] + K[i][1]*y[1];
            }

            // Joseph form: P = (I-KH)P(I-KH)^T + K R K^T
            double[][] KH = new double[4][4];
            for(int i=0;i<4;i++){
                KH[i][0]=K[i][0];
                KH[i][1]=K[i][1];
                // KH[i][2]=0; KH[i][3]=0
            }
            double[][] I = {
                {1,0,0,0},
                {0,1,0,0},
                {0,0,1,0},
                {0,0,0,1}
            };
            double[][] A = new double[4][4];
            for(int i=0;i<4;i++) for(int j=0;j<4;j++) A[i][j]=I[i][j]-KH[i][j];
            double[][] AP = mul44(A,P);
            double[][] APA = mul44(AP, T44(A));

            // KRK^T
            double[][] KR = new double[4][2];
            for(int i=0;i<4;i++){
                KR[i][0]=K[i][0]*R[0][0];
                KR[i][1]=K[i][1]*R[1][1];
            }
            double[][] KRKt = new double[4][4];
            for(int i=0;i<4;i++){
                for(int j=0;j<4;j++){
                    KRKt[i][j] = KR[i][0]*K[j][0] + KR[i][1]*K[j][1];
                }
            }

            P = add44(APA, KRKt);

            System.out.printf("k=%d  z=[%.3f, %.3f]  xhat=[%.3f, %.3f, %.3f, %.3f]%n",
                    k, z[0], z[1], x[0], x[1], x[2], x[3]);
        }
    }
}
