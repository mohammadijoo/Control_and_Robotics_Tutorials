import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.distribution.NormalDistribution;

public class ChanceConstraint {

    private static final NormalDistribution stdNormal =
        new NormalDistribution(0.0, 1.0);

    public static boolean isSatisfied(
            RealVector mu,
            RealMatrix Sigma,
            RealVector a,
            double b,
            double epsilon
    ) {
        double m = a.dotProduct(mu);
        RealVector Sigma_a = Sigma.operate(a);
        double s2 = a.dotProduct(Sigma_a);
        if (s2 < 0.0) {
            s2 = 0.0;
        }
        double s = Math.sqrt(s2);
        double beta = stdNormal.inverseCumulativeProbability(1.0 - epsilon);
        return m + beta * s <= b + 1e-9;
    }

    public static void main(String[] args) {
        // Example: 2D state
        double[][] SigmaData = {
            {0.05 * 0.05, 0.0},
            {0.0, 0.02 * 0.02}
        };
        RealMatrix Sigma = MatrixUtils.createRealMatrix(SigmaData);
        RealVector mu = MatrixUtils.createRealVector(new double[]{1.0, 0.2});
        RealVector a = MatrixUtils.createRealVector(new double[]{1.0, 0.0});
        double pMax = 2.0;
        double epsilon = 0.01;

        boolean ok = isSatisfied(mu, Sigma, a, pMax, epsilon);
        System.out.println("Chance constraint satisfied? " + ok);
    }
}
      
