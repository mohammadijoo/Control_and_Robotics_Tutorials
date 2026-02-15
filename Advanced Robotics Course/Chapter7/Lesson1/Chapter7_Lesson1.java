public class PlanarContact {

    public static double[][] frictionConeGenerators2D(double mu, int mEdges) {
        if (mu <= 0.0) {
            throw new IllegalArgumentException("mu must be positive");
        }
        double alpha = Math.atan(mu);
        double[] angles = new double[mEdges];
        double[][] Gc = new double[2][mEdges];

        for (int k = 0; k < mEdges; ++k) {
            double t = (double) k / (double) (mEdges - 1);
            angles[k] = -alpha + 2.0 * alpha * t;
            Gc[0][k] = Math.sin(angles[k]); // tangent
            Gc[1][k] = Math.cos(angles[k]); // normal
        }
        return Gc;
    }

    public static double[][] planarWrenchGenerators(
            double px, double py, double mu, int mEdges) {

        double[][] Gc = frictionConeGenerators2D(mu, mEdges);
        double[][] W = new double[3][mEdges];

        for (int j = 0; j < mEdges; ++j) {
            double fx = Gc[0][j];
            double fy = Gc[1][j];
            double tau = px * fy - py * fx;
            W[0][j] = fx;
            W[1][j] = fy;
            W[2][j] = tau;
        }
        return W;
    }
}
      
