// Simple first-order low-pass filter implemented manually
public class FirstOrderLPF {
    private final double b0;
    private final double a1;
    private double y1 = 0.0;

    public FirstOrderLPF(double Ts, double tau_f) {
        double denom = Ts + tau_f;
        this.b0 = Ts / denom;
        this.a1 = -Ts / denom;
    }

    public double filter(double u) {
        double y = -a1 * y1 + b0 * u;
        y1 = y;
        return y;
    }
}

// In a robot control loop:
// FirstOrderLPF gyroFilter = new FirstOrderLPF(0.005, 0.02);
// double filteredRate = gyroFilter.filter(rawGyroRate);
