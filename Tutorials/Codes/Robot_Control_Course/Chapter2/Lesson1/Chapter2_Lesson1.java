
import org.ejml.simple.SimpleMatrix;
import java.util.List;

class JointSample {
    public SimpleMatrix qd;  // n x 1
    public SimpleMatrix q;   // n x 1
}

public class JointSpaceMetrics {

    public static double computeISE(List<JointSample> traj, double dt) {
        double ise = 0.0;
        for (JointSample s : traj) {
            SimpleMatrix e = s.qd.minus(s.q);
            double norm2 = e.dot(e);  // squared Euclidean norm
            ise += norm2 * dt;
        }
        return ise;
    }

    public static void main(String[] args) {
        // In practice: acquire samples from your robot middleware
        // and feed them into computeISE.
    }
}
