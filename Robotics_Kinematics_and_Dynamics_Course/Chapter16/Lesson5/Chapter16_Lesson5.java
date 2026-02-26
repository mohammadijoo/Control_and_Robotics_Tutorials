public class StewartPlatform {
    private final double[][] B; // base points [6][3]
    private final double[][] P; // platform points [6][3]

    public StewartPlatform(double[][] basePoints, double[][] platformPoints) {
        if (basePoints.length != 6 || platformPoints.length != 6) {
            throw new IllegalArgumentException("Need 6 base and 6 platform points.");
        }
        this.B = basePoints;
        this.P = platformPoints;
    }

    public static double[][] rotZYX(double phi, double theta, double psi) {
        double cphi = Math.cos(phi), sphi = Math.sin(phi);
        double cth  = Math.cos(theta), sth  = Math.sin(theta);
        double cpsi = Math.cos(psi), spsi = Math.sin(psi);

        double[][] Rx = {
            {1.0, 0.0, 0.0},
            {0.0, cphi, -sphi},
            {0.0, sphi, cphi}
        };
        double[][] Ry = {
            {cth, 0.0, sth},
            {0.0, 1.0, 0.0},
            {-sth, 0.0, cth}
        };
        double[][] Rz = {
            {cpsi, -spsi, 0.0},
            {spsi, cpsi, 0.0},
            {0.0, 0.0, 1.0}
        };
        return matMul(Rz, matMul(Ry, Rx));
    }

    private static double[] matVec(double[][] M, double[] v) {
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
        }
        return r;
    }

    private static double[][] matMul(double[][] A, double[][] B) {
        double[][] R = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    R[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return R;
    }

    private static double[] vecSub(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double norm(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    // IK: pose = [px, py, pz, phi, theta, psi]
    public double[] inverseKinematics(double[] pose) {
        double px = pose[0], py = pose[1], pz = pose[2];
        double phi = pose[3], theta = pose[4], psi = pose[5];
        double[] p = new double[]{px, py, pz};
        double[][] R = rotZYX(phi, theta, psi);

        double[] L = new double[6];
        for (int i = 0; i < 6; ++i) {
            double[] Rp = matVec(R, P[i]);
            double[] di = vecSub(new double[]{p[0] + Rp[0], p[1] + Rp[1], p[2] + Rp[2]}, B[i]);
            L[i] = norm(di);
        }
        return L;
    }
}
      
