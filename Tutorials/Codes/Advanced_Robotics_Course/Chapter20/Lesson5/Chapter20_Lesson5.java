public class EpisodeMetrics {
    public int success;
    public double time;
    public double energy;
    public EpisodeMetrics(int success, double time, double energy) {
        this.success = success;
        this.time = time;
        this.energy = energy;
    }
}

public class EvalStats {
    public double pHat, pCiLow, pCiHigh;
    public double timeMean, timeStd;
    public double energyMean, energyStd;
}

public class Evaluator {

    public static EvalStats computeStats(java.util.List<EpisodeMetrics> data) {
        int N = data.size();
        double sumSuccess = 0.0;
        double sumTime = 0.0, sumTime2 = 0.0;
        double sumEnergy = 0.0, sumEnergy2 = 0.0;

        for (EpisodeMetrics e : data) {
            sumSuccess += (double)e.success;
            sumTime += e.time;
            sumTime2 += e.time * e.time;
            sumEnergy += e.energy;
            sumEnergy2 += e.energy * e.energy;
        }

        EvalStats out = new EvalStats();
        out.pHat = sumSuccess / (double)N;

        double z = 1.96;
        double seP = Math.sqrt(out.pHat * (1.0 - out.pHat) / (double)N);
        out.pCiLow = Math.max(0.0, out.pHat - z * seP);
        out.pCiHigh = Math.min(1.0, out.pHat + z * seP);

        out.timeMean = sumTime / (double)N;
        out.energyMean = sumEnergy / (double)N;

        if (N > 1) {
            double varTime =
                (sumTime2 - (double)N * out.timeMean * out.timeMean) / (double)(N - 1);
            double varEnergy =
                (sumEnergy2 - (double)N * out.energyMean * out.energyMean) / (double)(N - 1);
            out.timeStd = Math.sqrt(Math.max(0.0, varTime));
            out.energyStd = Math.sqrt(Math.max(0.0, varEnergy));
        }

        return out;
    }

    public static void main(String[] args) {
        java.util.List<EpisodeMetrics> log = new java.util.ArrayList<>();
        // TODO: populate log from your experiment runner
        EvalStats stats = computeStats(log);
        System.out.println("Success rate: " + stats.pHat +
                           " CI95: [" + stats.pCiLow + ", " + stats.pCiHigh + "]");
    }
}
      
