public class DelayMarginExample {

    // Compute delay margin from phase margin (deg) and gain crossover (rad/s)
    public static double delayMargin(double phaseMarginDeg, double wGc) {
        double phaseMarginRad = Math.toRadians(phaseMarginDeg);
        return phaseMarginRad / wGc;
    }

    public static void main(String[] args) {
        double phaseMarginDeg = 45.0; // from design tool or measurement
        double wGc = 5.0;             // rad/s
        double Td = delayMargin(phaseMarginDeg, wGc);
        System.out.println("Delay margin (s): " + Td);

        // In a robotics framework such as WPILib, this can guide the selection
        // of sampling times and network delays allowed in a closed-loop system.
    }
}
