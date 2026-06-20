public class FirstOrderServo {

    private final double tau;   // plant time constant
    private double K;           // proportional gain

    public FirstOrderServo(double tau) {
        this.tau = tau;
    }

    public void tuneForBandwidth(double omegaBTarget) {
        // K = tau * omega_b - 1
        this.K = tau * omegaBTarget - 1.0;
    }

    public double getGain() {
        return K;
    }

    public double getClosedLoopBandwidth() {
        return (1.0 + K) / tau;
    }

    public static void main(String[] args) {
        FirstOrderServo servo = new FirstOrderServo(0.1);
        servo.tuneForBandwidth(20.0);

        System.out.println("Proportional gain K = " + servo.getGain());
        System.out.println("Estimated bandwidth = " + servo.getClosedLoopBandwidth() + " rad/s");

        // In a ROSJava node, the gain K would be applied to the position or velocity error.
    }
}
