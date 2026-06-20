public class FilteredDerivativePID {
    private double Kp;
    private double Ki;
    private double Kd;
    private double N;
    private double dt;

    private double integral;
    private double ePrev;
    private double dFilt;
    private double alpha;

    public FilteredDerivativePID(double Kp, double Ki, double Kd,
                                 double N, double dt) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.N  = N;
        this.dt = dt;

        this.integral = 0.0;
        this.ePrev    = 0.0;
        this.dFilt    = 0.0;
        this.alpha    = Math.exp(-N * dt);
    }

    public double update(double reference, double measurement) {
        double e = reference - measurement;

        // integral term
        integral += e * dt;

        // filtered derivative term
        double dRaw = (e - ePrev) / dt;
        dFilt = alpha * dFilt + (1.0 - alpha) * dRaw;

        ePrev = e;

        return Kp * e + Ki * integral + Kd * dFilt;
    }
}
