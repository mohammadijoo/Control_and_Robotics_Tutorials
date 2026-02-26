public class LowPassPrefilter {
    private final double tauF;  // time constant
    private final double Ts;    // sampling period
    private double y;           // internal state

    public LowPassPrefilter(double tauF, double Ts) {
        this.tauF = tauF;
        this.Ts = Ts;
        this.y = 0.0;
    }

    // Update with new reference r_k and return filtered reference
    public double update(double r) {
        double alpha = Ts / tauF;
        y += alpha * (r - y);
        return y;
    }
}

// Example usage in a servo loop:
//
// LowPassPrefilter rFilter = new LowPassPrefilter(0.1, 0.002); // tauF = 0.1 s, Ts = 2 ms
//
// while (running) {
//     double rRaw = referenceGenerator.getNextCommand();
//     double rShaped = rFilter.update(rRaw);
//
//     double yMeas = robot.readJointPosition(jointIndex);
//     double control = pid.compute(rShaped, yMeas);
//     robot.writeJointTorque(jointIndex, control);
// }
