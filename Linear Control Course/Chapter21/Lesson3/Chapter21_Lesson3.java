public class PIController {
    private double Kp;
    private double Ki;
    private double integral;
    private double uMin;
    private double uMax;

    public PIController(double Kp, double Ki, double uMin, double uMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.uMin = uMin;
        this.uMax = uMax;
        this.integral = 0.0;
    }

    public double update(double ref, double y, double dt) {
        double e = ref - y;

        // Integrator
        integral += e * dt;

        double uInt = Ki * integral;
        double u = Kp * e + uInt;

        // Saturate
        if (u > uMax) {
            u = uMax;
        } else if (u < uMin) {
            u = uMin;
        }

        return u;
    }

    public void reset() {
        integral = 0.0;
    }
}

// Example of usage in a periodic control loop:
// PIController axisPI = new PIController(5.0, 10.0, -1.0, 1.0);
// double dt = 0.01; // 100 Hz
// while (true) {
//     double ref = referenceGenerator();
//     double y = sensorMeasurement();
//     double u = axisPI.update(ref, y, dt);
//     applyMotorVoltage(u);
//     Thread.sleep((long)(dt * 1000));
// }
