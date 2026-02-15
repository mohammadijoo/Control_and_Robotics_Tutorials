public final class RunMetrics {
    private double sumE2 = 0.0;
    private double sumAbsE = 0.0;
    private double sumU2 = 0.0;
    private double sumDt = 0.0;
    private long samples = 0L;

    public void accumulate(double e, double u, double dt) {
        sumE2 += e * e;
        sumAbsE += Math.abs(e) * dt;
        sumU2 += u * u * dt;
        sumDt += dt;
        samples++;
    }

    public double getRms() {
        if (samples == 0L) return 0.0;
        return Math.sqrt(sumE2 / (double) samples);
    }

    public double getIae() {
        return sumAbsE;
    }

    public double getU2() {
        return sumU2;
    }
}

// Example skeleton for a log-processing method
// logRows is a list of objects with fields t, r, y, u, runId
public static Map<Integer, RunMetrics> computeMetrics(List<LogRow> logRows) {
    Map<Integer, RunMetrics> map = new HashMap<>();
    Map<Integer, Double> lastTime = new HashMap<>();

    for (LogRow row : logRows) {
        int runId = row.getRunId();
        double t = row.getTime();
        double r = row.getReference();
        double y = row.getOutput();
        double u = row.getControl();

        double tPrev = lastTime.getOrDefault(runId, t);
        double dt = t - tPrev;
        lastTime.put(runId, t);

        RunMetrics m = map.computeIfAbsent(runId, k -> new RunMetrics());
        double e = r - y;
        m.accumulate(e, u, dt);
    }

    return map;
}
      
