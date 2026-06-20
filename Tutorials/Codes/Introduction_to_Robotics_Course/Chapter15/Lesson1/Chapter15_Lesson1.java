public class SpeedSeparationConfig {
    public final double reactionTime;   // T_r
    public final double maxDecel;       // a_max > 0
    public final double safetyMargin;   // d_safe

    public SpeedSeparationConfig(double reactionTime, double maxDecel, double safetyMargin) {
        if (maxDecel <= 0.0) {
            throw new IllegalArgumentException("maxDecel must be positive");
        }
        this.reactionTime = reactionTime;
        this.maxDecel = maxDecel;
        this.safetyMargin = safetyMargin;
    }
}

public class HazardMonitor {

    public static double stoppingDistance(double v, SpeedSeparationConfig cfg) {
        if (v < 0.0) v = 0.0;
        return v * cfg.reactionTime + 0.5 * v * v / cfg.maxDecel;
    }

    public static double safetyFunction(double d, double v, SpeedSeparationConfig cfg) {
        double dStop = stoppingDistance(v, cfg);
        return d - dStop - cfg.safetyMargin;
    }

    public static boolean hazardActive(double d, double v, SpeedSeparationConfig cfg) {
        return safetyFunction(d, v, cfg) < 0.0;
    }

    public static void emergencyStop() {
        // Implement robot-specific stop behavior here
    }

    public static void monitorStep(double d, double v, SpeedSeparationConfig cfg) {
        if (hazardActive(d, v, cfg)) {
            emergencyStop();
        }
    }
}
      
