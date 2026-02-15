public class LowPassFilter {
    private double alpha;
    private double yPrev;

    public LowPassFilter(double tauF, double Ts) {
        setParameters(tauF, Ts);
        this.yPrev = 0.0;
    }

    public void setParameters(double tauF, double Ts) {
        this.alpha = Math.exp(-Ts / tauF);
    }

    public double filter(double yMeas) {
        double yF = alpha * yPrev + (1.0 - alpha) * yMeas;
        yPrev = yF;
        return yF;
    }
}

// Example of integrating into a PID-based joint controller
public class JointController {
    private double Kp;
    private double Kd;
    private double Ts;
    private LowPassFilter measFilter;
    private double prevError;

    public JointController(double Kp, double Kd, double Ts,
                           double tauF) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ts = Ts;
        this.prevError = 0.0;
        this.measFilter = new LowPassFilter(tauF, Ts);
    }

    public double computeControl(double qRef, double qMeasRaw) {
        double qMeas = measFilter.filter(qMeasRaw);
        double error = qRef - qMeas;
        double dError = (error - prevError) / Ts;
        prevError = error;
        return Kp * error + Kd * dError;
    }
}
