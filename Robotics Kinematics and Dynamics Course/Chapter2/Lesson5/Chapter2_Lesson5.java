public class SE3LieGroup {

    public static double[][] hatSO3(double[] w) {
        double wx = w[0], wy = w[1], wz = w[2];
        return new double[][] {
            { 0.0,   -wz,   wy },
            {  wz,   0.0,  -wx },
            { -wy,    wx,  0.0 }
        };
    }

    public static double[][] hatSE3(double[] xi) {
        double[] w = new double[] { xi[0], xi[1], xi[2] };
        double[] v = new double[] { xi[3], xi[4], xi[5] };

        double[][] Xi = new double[4][4];
        double[][] W = hatSO3(w);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Xi[i][j] = W[i][j];
            }
            Xi[i][3] = v[i];
        }
        return Xi;
    }

    public static double[][] matMul3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                C[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    public static double[][] expSO3Rodrigues(double[] w, double theta) {
        double norm = Math.sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
        if (norm < 1e-9) {
            double[][] I = { {1,0,0}, {0,1,0}, {0,0,1} };
            return I;
        }
        double[] wh = { w[0]/norm, w[1]/norm, w[2]/norm };
        double[][] W = hatSO3(wh);
        double th = theta * norm;

        double[][] I = { {1,0,0}, {0,1,0}, {0,0,1} };
        double s = Math.sin(th);
        double c = Math.cos(th);

        // R = I + s*W + (1-c)*W^2
        double[][] W2 = matMul3(W, W);
        double[][] R = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = I[i][j] + s * W[i][j] + (1.0 - c) * W2[i][j];
            }
        }
        return R;
    }

    public static double[][] expSE3(double[] xi, double theta) {
        double[] w = { xi[0], xi[1], xi[2] };
        double[] v = { xi[3], xi[4], xi[5] };
        double wnorm = Math.sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);

        double[][] g = new double[4][4];
        for (int i = 0; i < 4; ++i) g[i][i] = 1.0;

        if (wnorm < 1e-9) {
            g[0][3] = v[0] * theta;
            g[1][3] = v[1] * theta;
            g[2][3] = v[2] * theta;
            return g;
        }

        double[] wh = { w[0]/wnorm, w[1]/wnorm, w[2]/wnorm };
        double[][] W = hatSO3(wh);
        double th = theta * wnorm;
        double[][] R = expSO3Rodrigues(w, theta);

        // Compute A = I*th + (1-cos th)*W + (th - sin th)*W^2
        double[][] I = { {1,0,0}, {0,1,0}, {0,0,1} };
        double[][] W2 = matMul3(W, W);
        double s = Math.sin(th);
        double c = Math.cos(th);

        double[][] A = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                A[i][j] = I[i][j] * th + (1.0 - c) * W[i][j] + (th - s) * W2[i][j];
            }
        }

        double[] p = new double[3];
        for (int i = 0; i < 3; ++i) {
            p[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                g[i][j] = R[i][j];
            }
            g[i][3] = p[i];
        }

        return g;
    }
}
      
