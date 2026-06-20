// Chapter 6 - Lesson 1: Belief as a Probability Distribution Over Pose
// Autonomous Mobile Robots (Control Engineering)
//
// This Java example uses Apache Commons Math (linear algebra) to compute a 3D Gaussian PDF,
// discretize it on a grid, normalize by cell volume, and compute expected pose with a
// circular mean for theta.
//
// Dependency (Maven):
//   <dependency>
//     <groupId>org.apache.commons</groupId>
//     <artifactId>commons-math3</artifactId>
//     <version>3.6.1</version>
//   </dependency>

import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class Chapter6_Lesson1 {

    @FunctionalInterface
    interface Index3 {
        int index(int i, int j, int k);
    }

    private static double wrapToPi(double a) {
        final double TWO_PI = 2.0 * Math.PI;
        a = (a + Math.PI) % TWO_PI;
        if (a < 0.0) a += TWO_PI;
        return a - Math.PI;
    }

    private static double gaussianPdf3(double[] x, double[] mu, RealMatrix Sigma) {
        double[] dx = new double[] { x[0] - mu[0], x[1] - mu[1], wrapToPi(x[2] - mu[2]) };

        RealMatrix invS = new LUDecomposition(Sigma).getSolver().getInverse();
        double detS = new LUDecomposition(Sigma).getDeterminant();
        double norm = 1.0 / Math.sqrt(Math.pow(2.0 * Math.PI, 3.0) * detS);

        // quadratic form dx' invS dx
        double q = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                q += dx[i] * invS.getEntry(i, j) * dx[j];
            }
        }
        return norm * Math.exp(-0.5 * q);
    }

    public static void main(String[] args) {
        double[] mu = new double[] { 2.0, -1.0, 0.7 };
        RealMatrix Sigma = MatrixUtils.createRealMatrix(new double[][] {
            { 0.2 * 0.2, 0.0, 0.0 },
            { 0.0, 0.3 * 0.3, 0.0 },
            { 0.0, 0.0, Math.pow(10.0 * Math.PI / 180.0, 2.0) }
        });

        double[] xTest = new double[] { 2.1, -1.2, 0.75 };
        System.out.println("pdf(x_test) = " + gaussianPdf3(xTest, mu, Sigma));

        int Nx = 101, Ny = 101, Nt = 121;
        double x0 = 1.0, x1 = 3.0;
        double y0 = -2.0, y1 = 0.0;
        double th0 = -Math.PI, th1 = Math.PI;

        double[] xs = new double[Nx];
        double[] ys = new double[Ny];
        double[] ths = new double[Nt];

        for (int i = 0; i < Nx; ++i) xs[i] = x0 + (x1 - x0) * i / (Nx - 1.0);
        for (int j = 0; j < Ny; ++j) ys[j] = y0 + (y1 - y0) * j / (Ny - 1.0);
        for (int k = 0; k < Nt; ++k) ths[k] = th0 + (th1 - th0) * k / Nt; // endpoint excluded

        double dx = xs[1] - xs[0];
        double dy = ys[1] - ys[0];
        double dth = ths[1] - ths[0];
        double cellVol = dx * dy * dth;

        double[] b = new double[Nx * Ny * Nt];
        Index3 index3 = (i, j, k) -> (i * Ny + j) * Nt + k;

        double Z = 0.0;
        for (int i = 0; i < Nx; ++i) {
            for (int j = 0; j < Ny; ++j) {
                for (int k = 0; k < Nt; ++k) {
                    double[] x = new double[] { xs[i], ys[j], ths[k] };
                    double val = gaussianPdf3(x, mu, Sigma);
                    b[index3.index(i, j, k)] = val;
                    Z += val * cellVol;
                }
            }
        }
        for (int t = 0; t < b.length; ++t) b[t] /= Z;

        double check = 0.0;
        for (double v : b) check += v * cellVol;
        System.out.println("grid normalization check: " + check);

        double ex = 0.0, ey = 0.0, esin = 0.0, ecos = 0.0;
        for (int i = 0; i < Nx; ++i) {
            for (int j = 0; j < Ny; ++j) {
                for (int k = 0; k < Nt; ++k) {
                    double w = b[index3.index(i, j, k)] * cellVol;
                    ex += xs[i] * w;
                    ey += ys[j] * w;
                    esin += Math.sin(ths[k]) * w;
                    ecos += Math.cos(ths[k]) * w;
                }
            }
        }
        double eth = Math.atan2(esin, ecos);
        System.out.println("grid E[pose] = [" + ex + ", " + ey + ", " + eth + "]");
    }
}
