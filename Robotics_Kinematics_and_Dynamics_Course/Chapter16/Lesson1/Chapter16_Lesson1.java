public class ThreeRPR {

    static class Pose {
        double x, y, phi;
        Pose(double x, double y, double phi) { this.x = x; this.y = y; this.phi = phi; }
    }

    static class Vec2 {
        double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
    }

    static Vec2[] B = new Vec2[] {
        new Vec2(0.0, 0.0),
        new Vec2(1.0, 0.0),
        new Vec2(0.5, Math.sqrt(3.0) / 2.0)
    };

    static double rp = 0.2;
    static Vec2[] P = new Vec2[] {
        new Vec2(rp * 1.0,              rp * 0.0),
        new Vec2(rp * (-0.5),           rp * (Math.sqrt(3.0) / 2.0)),
        new Vec2(rp * (-0.5),           rp * (-Math.sqrt(3.0) / 2.0))
    };

    static Vec2 rot2(Vec2 v, double phi) {
        double c = Math.cos(phi);
        double s = Math.sin(phi);
        return new Vec2(c * v.x - s * v.y, s * v.x + c * v.y);
    }

    static double[] ik3RPR(Pose pose) {
        double[] L = new double[3];
        for (int i = 0; i < 3; ++i) {
            Vec2 PiBase = rot2(P[i], pose.phi);
            double cx = pose.x + PiBase.x;
            double cy = pose.y + PiBase.y;
            double px = cx - B[i].x;
            double py = cy - B[i].y;
            L[i] = Math.sqrt(px * px + py * py);
        }
        return L;
    }

    static double[] fkResidual(Pose pose, double[] L) {
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            Vec2 PiBase = rot2(P[i], pose.phi);
            double cx = pose.x + PiBase.x;
            double cy = pose.y + PiBase.y;
            double px = cx - B[i].x;
            double py = cy - B[i].y;
            double norm2 = px * px + py * py;
            r[i] = norm2 - L[i] * L[i];
        }
        return r;
    }

    static double[][] fkJacobian(Pose pose) {
        double[][] J = new double[3][3];
        double c = Math.cos(pose.phi);
        double s = Math.sin(pose.phi);
        for (int i = 0; i < 3; ++i) {
            Vec2 Pi = P[i];
            double pix = c * Pi.x - s * Pi.y;
            double piy = s * Pi.x + c * Pi.y;
            double cx = pose.x + pix;
            double cy = pose.y + piy;
            double px = cx - B[i].x;
            double py = cy - B[i].y;
            double dpxdphi = -s * Pi.x - c * Pi.y;
            double dpydphi =  c * Pi.x - s * Pi.y;

            double dphidx   = 2.0 * px;
            double dphidy   = 2.0 * py;
            double dphidphi = 2.0 * (px * dpxdphi + py * dpydphi);

            J[i][0] = dphidx;
            J[i][1] = dphidy;
            J[i][2] = dphidphi;
        }
        return J;
    }

    static boolean solve3x3(double[][] A, double[] b, double[] x) {
        double detA =
            A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
            A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
            A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
        if (Math.abs(detA) < 1e-12) return false;

        double[][] Ax = new double[3][3];
        double[][] Ay = new double[3][3];
        double[][] Aphi = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Ax[i][j] = A[i][j];
                Ay[i][j] = A[i][j];
                Aphi[i][j] = A[i][j];
            }
        }
        for (int i = 0; i < 3; ++i) {
            Ax[i][0] = b[i];
            Ay[i][1] = b[i];
            Aphi[i][2] = b[i];
        }

        double detAx =
            Ax[0][0]*(Ax[1][1]*Ax[2][2] - Ax[1][2]*Ax[2][1]) -
            Ax[0][1]*(Ax[1][0]*Ax[2][2] - Ax[1][2]*Ax[2][0]) +
            Ax[0][2]*(Ax[1][0]*Ax[2][1] - Ax[1][1]*Ax[2][0]);

        double detAy =
            Ay[0][0]*(Ay[1][1]*Ay[2][2] - Ay[1][2]*Ay[2][1]) -
            Ay[0][1]*(Ay[1][0]*Ay[2][2] - Ay[1][2]*Ay[2][0]) +
            Ay[0][2]*(Ay[1][0]*Ay[2][1] - Ay[1][1]*Ay[2][0]);

        double detAphi =
            Aphi[0][0]*(Aphi[1][1]*Aphi[2][2] - Aphi[1][2]*Aphi[2][1]) -
            Aphi[0][1]*(Aphi[1][0]*Aphi[2][2] - Aphi[1][2]*Aphi[2][0]) +
            Aphi[0][2]*(Aphi[1][0]*Aphi[2][1] - Aphi[1][1]*Aphi[2][0]);

        x[0] = detAx / detA;
        x[1] = detAy / detA;
        x[2] = detAphi / detA;
        return true;
    }

    static boolean fk3RPRNewton(double[] L, Pose pose, int maxIter, double tol) {
        for (int k = 0; k < maxIter; ++k) {
            double[] r = fkResidual(pose, L);
            double norm = Math.sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
            if (norm < tol) return true;

            double[][] J = fkJacobian(pose);
            double[] rhs = new double[] { -r[0], -r[1], -r[2] };
            double[] dx = new double[3];
            if (!solve3x3(J, rhs, dx)) return false;
            pose.x   += dx[0];
            pose.y   += dx[1];
            pose.phi += dx[2];
        }
        return false;
    }

    public static void main(String[] args) {
        Pose poseTrue = new Pose(0.2, 0.1, 0.3);
        double[] L = ik3RPR(poseTrue);
        Pose guess = new Pose(0.0, 0.0, 0.0);
        boolean ok = fk3RPRNewton(L, guess, 50, 1e-10);
        System.out.println("Converged: " + ok);
        System.out.println("True pose: " + poseTrue.x + ", " + poseTrue.y + ", " + poseTrue.phi);
        System.out.println("FK pose:   " + guess.x + ", " + guess.y + ", " + guess.phi);
    }
}
      
