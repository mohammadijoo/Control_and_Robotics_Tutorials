public class Planar2RIKLab {

    public static class Vec2 {
        public double x, y;
        public Vec2(double x, double y) { this.x = x; this.y = y; }
        public Vec2 plus(Vec2 other) { return new Vec2(x + other.x, y + other.y); }
        public Vec2 minus(Vec2 other) { return new Vec2(x - other.x, y - other.y); }
        public Vec2 scale(double s) { return new Vec2(s * x, s * y); }
        public double norm() { return Math.sqrt(x * x + y * y); }
    }

    public static class Robot2R {
        public double L1, L2;
        public double[] qMin = {-Math.PI, -Math.PI};
        public double[] qMax = { Math.PI,  Math.PI};

        public Robot2R(double L1, double L2) {
            this.L1 = L1;
            this.L2 = L2;
        }

        public Vec2 fk(double[] q) {
            double q1 = q[0], q2 = q[1];
            double x = L1 * Math.cos(q1) + L2 * Math.cos(q1 + q2);
            double y = L1 * Math.sin(q1) + L2 * Math.sin(q1 + q2);
            return new Vec2(x, y);
        }

        public double[][] jacobian(double[] q) {
            double q1 = q[0], q2 = q[1];
            double s1 = Math.sin(q1);
            double c1 = Math.cos(q1);
            double s12 = Math.sin(q1 + q2);
            double c12 = Math.cos(q1 + q2);
            double[][] J = new double[2][2];
            J[0][0] = -L1 * s1 - L2 * s12;
            J[0][1] = -L2 * s12;
            J[1][0] =  L1 * c1 + L2 * c12;
            J[1][1] =  L2 * c12;
            return J;
        }

        public double[] clip(double[] q) {
            double[] out = new double[2];
            for (int i = 0; i < 2; ++i) {
                out[i] = Math.min(Math.max(q[i], qMin[i]), qMax[i]);
            }
            return out;
        }
    }

    // Solve (H) delta = g for 2x2 symmetric positive definite H
    public static double[] solve2x2(double[][] H, double[] g) {
        double a = H[0][0], b = H[0][1], c = H[1][1];
        double det = a * c - b * b;
        double inv00 =  c / det;
        double inv01 = -b / det;
        double inv11 =  a / det;
        double d0 = inv00 * g[0] + inv01 * g[1];
        double d1 = inv01 * g[0] + inv11 * g[1];
        return new double[]{d0, d1};
    }

    public static class IKResult {
        public double[] qStar;
        public boolean success;
        public int iters;
    }

    public static IKResult ikDLS(Robot2R robot,
                                 Vec2 xDes,
                                 double[] q0,
                                 double lambdaInit,
                                 double epsTask,
                                 int maxIters,
                                 double stepMax) {
        double[] q = robot.clip(q0);
        double lambda = lambdaInit;

        for (int k = 0; k < maxIters; ++k) {
            Vec2 x = robot.fk(q);
            Vec2 e = new Vec2(xDes.x - x.x, xDes.y - x.y);
            double err = e.norm();
            if (err <= epsTask) {
                IKResult res = new IKResult();
                res.qStar = q;
                res.success = true;
                res.iters = k;
                return res;
            }

            double[][] J = robot.jacobian(q);
            // H = J^T J + lambda^2 I
            double JTJ00 = J[0][0] * J[0][0] + J[1][0] * J[1][0];
            double JTJ01 = J[0][0] * J[0][1] + J[1][0] * J[1][1];
            double JTJ11 = J[0][1] * J[0][1] + J[1][1] * J[1][1];

            double[][] H = new double[2][2];
            H[0][0] = JTJ00 + lambda * lambda;
            H[0][1] = JTJ01;
            H[1][0] = JTJ01;
            H[1][1] = JTJ11 + lambda * lambda;

            // g = J^T e
            double[] g = new double[2];
            g[0] = J[0][0] * e.x + J[1][0] * e.y;
            g[1] = J[0][1] * e.x + J[1][1] * e.y;

            double[] delta = solve2x2(H, g);
            double stepNorm = Math.sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
            if (stepNorm > stepMax) {
                double scale = stepMax / stepNorm;
                delta[0] *= scale;
                delta[1] *= scale;
            }

            double[] qNew = new double[2];
            qNew[0] = q[0] + delta[0];
            qNew[1] = q[1] + delta[1];
            qNew = robot.clip(qNew);

            // simple damping adaptation
            q = qNew;
        }

        IKResult res = new IKResult();
        res.qStar = q;
        res.success = false;
        res.iters = maxIters;
        return res;
    }

    public static void main(String[] args) {
        Robot2R robot = new Robot2R(0.5, 0.4);
        Vec2 xDes = new Vec2(0.6, 0.3);
        double[] q0 = {0.0, 0.0};

        IKResult res = ikDLS(robot, xDes, q0, 1e-2, 1e-4, 100, 0.2);
        System.out.println("Success: " + res.success);
        System.out.println("q_star: [" + res.qStar[0] + ", " + res.qStar[1] + "]");
        Vec2 xAch = robot.fk(res.qStar);
        System.out.println("achieved x: [" + xAch.x + ", " + xAch.y + "]");
    }
}
      
