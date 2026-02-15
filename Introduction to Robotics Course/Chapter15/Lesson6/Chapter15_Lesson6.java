public final class SafetyEnvelope1D {
    private final double Tr;
    private final double amax;
    private final double dng;
    private final double margin;

    public SafetyEnvelope1D(double reactionTime,
                            double maxDecel,
                            double noGoDistance,
                            double margin) {
        this.Tr = reactionTime;
        this.amax = maxDecel;
        this.dng = noGoDistance;
        this.margin = margin;
    }

    public double stoppingDistance(double v) {
        // d_stop = v*Tr + v^2/(2*a_max)
        return v * Tr + (v * v) / (2.0 * amax);
    }

    public boolean isSafe(double distance, double v) {
        if (v < 0.0) {
            return true; // moving away
        }
        double dAvailable = distance - dng - margin;
        if (dAvailable < 0.0) {
            dAvailable = 0.0;
        }
        double dRequired = stoppingDistance(v);
        return dRequired <= dAvailable;
    }

    public static void main(String[] args) {
        SafetyEnvelope1D env = new SafetyEnvelope1D(0.2, 3.0, 0.4, 0.05);
        double distance = 1.0;
        double v = 0.8;
        if (env.isSafe(distance, v)) {
            System.out.println("Command accepted (Java).");
        } else {
            System.out.println("Safety stop (Java).");
        }
    }
}
      
