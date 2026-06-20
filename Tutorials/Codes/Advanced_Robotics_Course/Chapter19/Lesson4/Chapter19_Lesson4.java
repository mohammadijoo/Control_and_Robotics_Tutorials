public interface WorldModel {
    // Predict next latent state z_{t+1} from z_t and action a_t
    double[] predictNext(double[] z, double[] a);
}

public interface Policy {
    // Choose action a_t given z_t
    double[] act(double[] z);
}

public class LinearWorldModel implements WorldModel {
    private final double[][] A;
    private final double[][] B;

    public LinearWorldModel(double[][] A, double[][] B) {
        this.A = A;
        this.B = B;
    }

    @Override
    public double[] predictNext(double[] z, double[] a) {
        int n = A.length;
        double[] next = new double[n];
        for (int i = 0; i != n; ++i) {
            double sum = 0.0;
            for (int j = 0; j != z.length; ++j) {
                sum += A[i][j] * z[j];
            }
            for (int k = 0; k != a.length; ++k) {
                sum += B[i][k] * a[k];
            }
            next[i] = sum;
        }
        return next;
    }
}

// Example proportional policy toward target position
public class ProportionalPolicy implements Policy {
    private final double xStar;
    private final double kP;

    public ProportionalPolicy(double xStar, double kP) {
        this.xStar = xStar;
        this.kP = kP;
    }

    @Override
    public double[] act(double[] z) {
        double x = z[0];
        double error = xStar - x;
        double a = kP * error;
        return new double[]{a};
    }
}
      
