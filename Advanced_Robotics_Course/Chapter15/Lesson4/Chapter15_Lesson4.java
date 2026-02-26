import java.util.Random;

public class SwarmConsensus {
    private final int N;
    private final int steps;
    private final double pFail;
    private final double pDrop;
    private final double eps;
    private final double noiseStd;
    private final double[] x;
    private final boolean[] alive;
    private final Random rng;

    public SwarmConsensus(int N, int steps,
                          double pFail, double pDrop,
                          double eps, double noiseStd) {
        this.N = N;
        this.steps = steps;
        this.pFail = pFail;
        this.pDrop = pDrop;
        this.eps = eps;
        this.noiseStd = noiseStd;
        this.x = new double[N];
        this.alive = new boolean[N];
        this.rng = new Random(0L);

        for (int i = 0; i < N; i++) {
            x[i] = -1.0 + 2.0 * rng.nextDouble();
            alive[i] = (rng.nextDouble() >= pFail);
        }
    }

    private double gauss() {
        // Box-Muller
        double u1 = rng.nextDouble();
        double u2 = rng.nextDouble();
        return Math.sqrt(-2.0 * Math.log(u1))
             * Math.cos(2.0 * Math.PI * u2) * noiseStd;
    }

    public void run() {
        for (int k = 0; k < steps; k++) {
            double[] xNew = x.clone();
            for (int i = 0; i < N; i++) {
                if (!alive[i]) continue;

                int left = (i - 1 + N) % N;
                int right = (i + 1) % N;

                double sumDiff = 0.0;
                int count = 0;

                if (rng.nextDouble() >= pDrop && alive[left]) {
                    sumDiff += x[left] - x[i];
                    count++;
                }
                if (rng.nextDouble() >= pDrop && alive[right]) {
                    sumDiff += x[right] - x[i];
                    count++;
                }

                if (count > 0) {
                    double avgDiff = sumDiff / (double) count;
                    xNew[i] = x[i] + eps * avgDiff + gauss();
                }
            }
            System.arraycopy(xNew, 0, x, 0, N);
        }
    }

    public double computeRMSE() {
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < N; i++) {
            if (alive[i]) {
                sum += x[i];
                count++;
            }
        }
        if (count == 0) return 0.0;
        double mean = sum / (double) count;

        double err2 = 0.0;
        for (int i = 0; i < N; i++) {
            if (alive[i]) {
                double d = x[i] - mean;
                err2 += d * d;
            }
        }
        return Math.sqrt(err2 / (double) count);
    }

    public static void main(String[] args) {
        SwarmConsensus sim = new SwarmConsensus(100, 300, 0.2, 0.1, 0.2, 0.01);
        sim.run();
        System.out.println("RMSE = " + sim.computeRMSE());
    }
}
      
