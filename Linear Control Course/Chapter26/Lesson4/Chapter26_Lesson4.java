import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;

public class VelocityController {
  private final PIDController pid;
  private final LinearFilter measFilter;

  public VelocityController(double kp, double ki, double kd, double cutoffHz, double dt) {
    pid = new PIDController(kp, ki, kd);
    // Simple single-pole IIR low-pass design expressed as a 1-tap FIR here
    double alpha = Math.exp(-2.0 * Math.PI * cutoffHz * dt);
    // Equivalent first-order IIR: y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    // WPILib LinearFilter.singlePoleIIR(a, dt) can also be used.
    measFilter = LinearFilter.singlePoleIIR(cutoffHz, dt);
  }

  public double calculate(double setpoint, double encoderRate) {
    double filteredRate = measFilter.calculate(encoderRate); // measurement-path
    return pid.calculate(filteredRate, setpoint);
  }
}
