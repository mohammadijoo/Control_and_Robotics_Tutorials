public class DelayRobustnessMonitor {

    // Phase margin and crossover frequency obtained from offline Bode analysis
    private final double phaseMarginDeg; // e.g., 55.0
    private final double omegaC;         // rad/s, e.g., 12.0
    private final double delayMargin;    // seconds

    // Safety factor: maximum allowed delay as a fraction of delay margin
    private final double safetyFactor;   // e.g., 0.6 (60%)

    public DelayRobustnessMonitor(double phaseMarginDeg, double omegaC,
                                  double safetyFactor) {
        this.phaseMarginDeg = phaseMarginDeg;
        this.omegaC = omegaC;
        this.safetyFactor = safetyFactor;

        double phaseMarginRad = Math.toRadians(phaseMarginDeg);
        this.delayMargin = phaseMarginRad / omegaC;
    }

    public double getDelayMargin() {
        return delayMargin;
    }

    public boolean isDelayAcceptable(double estimatedDelay) {
        // estimatedDelay: current worst-case I/O + computation delay (seconds)
        double maxAllowed = safetyFactor * delayMargin;
        return estimatedDelay <= maxAllowed;
    }

    public static void main(String[] args) {
        DelayRobustnessMonitor monitor =
            new DelayRobustnessMonitor(55.0, 12.0, 0.6);

        double estimatedDelay = 0.020; // 20 ms, for example
        System.out.println("Delay margin [s] = " + monitor.getDelayMargin());
        if (monitor.isDelayAcceptable(estimatedDelay)) {
            System.out.println("Delay is acceptable for high-performance mode.");
        } else {
            System.out.println("Delay too large: switch to conservative gains.");
        }
    }
}
