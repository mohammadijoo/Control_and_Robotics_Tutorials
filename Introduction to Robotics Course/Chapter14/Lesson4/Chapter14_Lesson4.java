public class TrustFilter {
    public static double[] updateTrust(int[] successes, double lambda, double T0) {
        double T = T0;
        double[] history = new double[successes.length + 1];
        history[0] = T;
        for (int k = 0; k < successes.length; ++k) {
            T = (1.0 - lambda) * T + lambda * (double) successes[k];
            history[k + 1] = T;
        }
        return history;
    }

    public static void main(String[] args) {
        int[] seq = {1, 1, 0, 1, 1, 1, 0, 1, 1, 1};
        double lambda = 0.3;
        double T0 = 0.2;
        double[] trustTraj = updateTrust(seq, lambda, T0);

        for (int k = 0; k < trustTraj.length; ++k) {
            System.out.printf("Step %d: T = %.3f%n", k, trustTraj[k]);
        }
    }
}
      
