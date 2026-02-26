import java.util.List;

public class LinearIrl {

    // phi(s,a) is assumed known and returns double[k]
    public static double[] phi(int s, int a, int k) {
        double[] f = new double[k];
        // TODO: fill with task-specific features
        return f;
    }

    public static double[] empiricalFeatureExpectation(List<int[][]> demos,
                                                       double gamma, int k) {
        double[] muE = new double[k];
        for (int[][] traj : demos) {
            double discount = 1.0;
            for (int t = 0; t < traj.length; t++) {
                int s = traj[t][0];
                int a = traj[t][1];
                double[] f = phi(s, a, k);
                for (int i = 0; i < k; i++) {
                    muE[i] += discount * f[i];
                }
                discount *= gamma;
            }
        }
        int n = demos.size();
        for (int i = 0; i < k; i++) {
            muE[i] /= (double) n;
        }
        return muE;
    }

    public static void gradientUpdate(double[] theta,
                                      double[] muE,
                                      double[] muTheta,
                                      double alpha) {
        for (int i = 0; i < theta.length; i++) {
            double grad = muE[i] - muTheta[i];
            theta[i] += alpha * grad;
        }
    }
}
      
