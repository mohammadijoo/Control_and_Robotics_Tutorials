import edu.wpi.first.math.controller.PIDController;

public class JointServo {
  private final PIDController pid;

  public JointServo() {
    // Gains tuned offline via Bode/Nyquist/Nichols
    double kp = 40.0;
    double ki = 0.0;
    double kd = 0.0;
    pid = new PIDController(kp, ki, kd);
    pid.setTolerance(0.01); // rad/s tolerance, for example
  }

  public double update(double setpoint, double measurement) {
    pid.setSetpoint(setpoint);
    double output = pid.calculate(measurement);
    return output; // send to motor drive
  }
}
