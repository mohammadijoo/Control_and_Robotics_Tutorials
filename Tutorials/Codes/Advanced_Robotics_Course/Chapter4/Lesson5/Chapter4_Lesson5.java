public class PointTrajectoryOptimizer {

    static class Path {
        double[][] p; // shape (N_plus_1, 2)
    }

    static double[] circleCenter = new double[]{0.5, 0.0};
    static double circleRadius = 0.25;
    static double clearance = 0.05;

    static double smoothnessCostAndGrad(Path path, double[][] grad) {
        int N_plus_1 = path.p.length;
        double cost = 0.0;

        for (int k = 1; k < N_plus_1 - 1; ++k) {
            double ddx = path.p[k + 1][0] - 2.0 * path.p[k][0] + path.p[k - 1][0];
            double ddy = path.p[k + 1][1] - 2.0 * path.p[k][1] + path.p[k - 1][1];

            double ddNorm2 = ddx * ddx + ddy * ddy;
            cost += 0.5 * ddNorm2;

            grad[k - 1][0] += -ddx;
            grad[k - 1][1] += -ddy;

            grad[k][0]     +=  2.0 * ddx;
            grad[k][1]     +=  2.0 * ddy;

            grad[k + 1][0] += -ddx;
            grad[k + 1][1] += -ddy;
        }
        return cost;
    }

    static double collisionCostAndGrad(Path path, double[][] grad) {
        int N_plus_1 = path.p.length;
        double cost = 0.0;

        for (int k = 1; k < N_plus_1 - 1; ++k) {
            double dx = path.p[k][0] - circleCenter[0];
            double dy = path.p[k][1] - circleCenter[1];
            double norm = Math.sqrt(dx * dx + dy * dy);
            double d = norm - circleRadius;

            if (d < clearance) {
                double phi = 0.5 * (clearance - d) * (clearance - d);
                cost += phi;
                double dphi_dd = -(clearance - d);

                if (norm > 1e-6) {
                    double nx = dx / norm;
                    double ny = dy / norm;

                    // dd/dx = nx, dd/dy = ny
                    grad[k][0] += dphi_dd * nx;
                    grad[k][1] += dphi_dd * ny;
                }
            }
        }
        return cost;
    }

    public static void main(String[] args) {
        int N = 40;
        Path path = new Path();
        path.p = new double[N + 1][2];

        double[] start = new double[]{-0.5, 0.0};
        double[] goal  = new double[]{ 1.0, 0.0};

        for (int k = 0; k <= N; ++k) {
            double alpha = (double) k / N;
            path.p[k][0] = (1.0 - alpha) * start[0] + alpha * goal[0];
            path.p[k][1] = (1.0 - alpha) * start[1] + alpha * goal[1];
        }

        double lambdaSmooth = 1.0;
        double lambdaCol = 10.0;
        double alphaStep = 0.01;

        for (int it = 0; it < 200; ++it) {
            double[][] grad = new double[N + 1][2];

            double Js = smoothnessCostAndGrad(path, grad);
            double Jc = collisionCostAndGrad(path, grad);

            // Gradient descent on internal points
            for (int k = 1; k < N; ++k) {
                path.p[k][0] -= alphaStep * (lambdaSmooth * grad[k][0] + lambdaCol * grad[k][0]);
                path.p[k][1] -= alphaStep * (lambdaSmooth * grad[k][1] + lambdaCol * grad[k][1]);
            }

            if (it % 20 == 0) {
                System.out.println("Iter " + it + ": Js = " + Js + ", Jc = " + Jc);
            }
        }
    }
}
      
