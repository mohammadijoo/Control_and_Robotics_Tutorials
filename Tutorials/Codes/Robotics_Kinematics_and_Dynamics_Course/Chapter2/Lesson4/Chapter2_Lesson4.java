public class LieGroups {

    public static double[][] hatOmega(double[] w) {
        double wx = w[0], wy = w[1], wz = w[2];
        return new double[][]{
            {0.0,   -wz,   wy},
            {wz,    0.0,  -wx},
            {-wy,   wx,   0.0}
        };
    }

    public static double[] veeOmega(double[][] W) {
        return new double[]{
            W[2][1],
            W[0][2],
            W[1][0]
        };
    }

    public static double[][] so3Exp(double[] omega, double theta) {
        double n = norm(omega);
        if (n < 1e-9) {
            return identity3();
        }
        double[] w = scale(omega, 1.0 / n);
        double[][] W = hatOmega(w);
        double[][] W2 = matMul3(W, W);
        double[][] I = identity3();

        double[][] R = add3(
            add3(I, scale3(W, Math.sin(theta))),
            scale3(W2, 1.0 - Math.cos(theta))
        );
        return R;
    }

    public static So3LogResult so3Log(double[][] R) {
        double trace = R[0][0] + R[1][1] + R[2][2];
        double cosTheta = (trace - 1.0) / 2.0;
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
        double theta = Math.acos(cosTheta);
        if (theta < 1e-9) {
            return new So3LogResult(new double[]{0.0, 0.0, 0.0}, 0.0);
        }
        double[][] diff = sub3(R, transpose3(R));
        double factor = theta / (2.0 * Math.sin(theta));
        double[][] W = scale3(diff, factor);
        double[] omega = veeOmega(W);
        double n = norm(omega);
        omega = scale(omega, 1.0 / n);
        return new So3LogResult(omega, theta);
    }

    public static Se3LogResult se3Log(double[][] T) {
        double[][] R = new double[3][3];
        double[] p = new double[3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = T[i][j];
            }
            p[i] = T[i][3];
        }
        So3LogResult logR = so3Log(R);
        double[] omega = logR.omega;
        double theta = logR.theta;

        double[] v;
        if (theta < 1e-9) {
            theta = norm(p);
            if (theta < 1e-9) {
                v = new double[]{0.0, 0.0, 0.0};
                omega = new double[]{0.0, 0.0, 0.0};
                return new Se3LogResult(v, omega, 0.0);
            }
            v = scale(p, 1.0 / theta);
            omega = new double[]{0.0, 0.0, 0.0};
            return new Se3LogResult(v, omega, theta);
        }

        double[][] W = hatOmega(omega);
        double[][] W2 = matMul3(W, W);
        double[][] I = identity3();

        double c = Math.cos(theta);
        double s = Math.sin(theta);
        double[][] Jinv = add3(
            add3(I, scale3(W, -0.5)),
            scale3(
                W2,
                (1.0 / (theta * theta)) - (1.0 + c) / (2.0 * theta * s)
            )
        );

        v = matVec3(Jinv, p);
        return new Se3LogResult(v, omega, theta);
    }

    // Helper classes and vector/matrix utility functions omitted for brevity:
    // norm, scale, identity3, matMul3, matVec3, add3, sub3, transpose3, scale3, etc.
    // They should implement standard 3x3 operations.
}
      
