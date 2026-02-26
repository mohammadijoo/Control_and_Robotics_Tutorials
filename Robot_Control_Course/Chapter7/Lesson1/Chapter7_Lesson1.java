
public interface Disturbance {
    double value(double t, double q, double qdot);
}

public class JointModel {
    // Nominal parameters
    private final double I;
    private final double b;
    private final double mgl;

    private Disturbance disturbance;

    public JointModel(double I, double b, double mgl) {
        this.I = I;
        this.b = b;
        this.mgl = mgl;
        this.disturbance = (t, q, qdot) -> 0.0; // default: no disturbance
    }

    public void setDisturbance(Disturbance d) {
        this.disturbance = d;
    }

    public double[] f(double t, double[] x, double tau) {
        // x = [q, qdot]
        double q = x[0];
        double qdot = x[1];

        double d = disturbance.value(t, q, qdot); // matched disturbance
        double qddot = (tau - b * qdot - mgl * Math.sin(q) + d) / I;

        return new double[]{qdot, qddot};
    }
}
