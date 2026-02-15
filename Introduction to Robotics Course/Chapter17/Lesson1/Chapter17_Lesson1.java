import java.util.Random;

public class WorkcellSim {

    public static class Stats {
        public double avgQueueLength;
        public double throughput;
        public double utilization;
    }

    public static Stats simulateWorkcell(int K, double lam) {
        Random rng = new Random(42L);
        int q = 0;
        int totalDepartures = 0;
        long totalQ = 0L;

        for (int k = 0; k < K; ++k) {
            int arrivals = (rng.nextDouble() < lam) ? 1 : 0;
            q += arrivals;

            if (q > 0) {
                q -= 1;
                totalDepartures += 1;
            }

            totalQ += q;
        }

        Stats s = new Stats();
        s.avgQueueLength = (double) totalQ / K;
        s.throughput = (double) totalDepartures / K;
        s.utilization = s.throughput; // capacity = 1 job/step
        return s;
    }

    public static void main(String[] args) {
        Stats s = simulateWorkcell(10000, 0.7);
        System.out.println("Average queue length: " + s.avgQueueLength);
        System.out.println("Throughput: " + s.throughput);
        System.out.println("Utilization: " + s.utilization);
    }
}
      
