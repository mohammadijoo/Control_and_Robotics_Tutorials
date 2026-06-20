import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;

public class JointControl {
    private final PIDController pid;
    private final ArmFeedforward ff;

    public JointControl(double kp, double ki, double kd,
                        double ks, double kg, double kv, double ka) {
        pid = new PIDController(kp, ki, kd);
        // ks: static friction, kg: gravity term, kv: velocity, ka: acceleration
        ff  = new ArmFeedforward(ks, kg, kv, ka);
    }

    public double update(double refAngle,
                         double refVelocity,
                         double refAcceleration,
                         double measuredAngle,
                         double dt) {
        // Feedback term on joint angle error
        double u_fb = pid.calculate(measuredAngle, refAngle);

        // Feedforward term based on known disturbance model (e.g., gravity torque)
        double u_ff = ff.calculate(refAngle, refVelocity, refAcceleration);

        // Total applied voltage or torque command
        return u_fb + u_ff;
    }
}
