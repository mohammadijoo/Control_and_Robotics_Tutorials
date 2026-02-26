
public class LearnedPdJointController {
    // Example: gain scheduling table learned from data
    // (e.g., Map from velocity bin to extra damping term)
    private final double[] velBins = {-5.0, -1.0, 1.0, 5.0};
    private final double[] extraKd = {4.0, 1.0, 1.0, 4.0};

    private double baseKp = 50.0;
    private double baseKd = 10.0;

    private double lookupExtraKd(double qd) {
        // naive piecewise-constant schedule
        for (int i = 0; i < velBins.length - 1; ++i) {
            if (qd >= velBins[i] && qd < velBins[i + 1]) {
                return extraKd[i];
            }
        }
        return extraKd[extraKd.length - 1];
    }

    public double computeTorque(double q, double qd,
                                double qRef, double qdRef, double qddRef)
    {
        double qTilde = q - qRef;
        double qdTilde = qd - qdRef;
        double KdEff = baseKd + lookupExtraKd(qd); // learned scheduling
        double v = qddRef - KdEff * qdTilde - baseKp * qTilde;
        double M = 1.2; // nominal
        return M * v;   // ignoring Coriolis and gravity in this sketch
    }
}
