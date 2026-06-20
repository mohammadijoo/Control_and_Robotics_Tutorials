public class ConsensusSimulation {
    public static void main(String[] args) {
        int N = 4;
        double alpha = 0.3;

        // Build Laplacian for complete graph
        double[][] L = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                if (i == j) {
                    L[i][j] = N - 1; // degree = N - 1
                } else {
                    L[i][j] = -1.0;
                }
            }
        }

        // Initial states
        double[] x = new double[] {0.0, 2.0, -1.0, 4.0};
        int T = 30;

        for (int k = 0; k < T; k++) {
            double[] xNext = new double[N];
            for (int i = 0; i < N; i++) {
                double sum = 0.0;
                for (int j = 0; j < N; j++) {
                    sum += L[i][j] * x[j];
                }
                xNext[i] = x[i] - alpha * sum;
            }
            x = xNext;
        }

        System.out.println("Final state:");
        for (int i = 0; i < N; i++) {
            System.out.print(x[i] + " ");
        }
        System.out.println();

        double avg0 = (0.0 + 2.0 - 1.0 + 4.0) / 4.0;
        System.out.println("Average of initial states: " + avg0);
    }
}
      
