
import java.util.Arrays;

public class RobotSubfieldsDemo {
    public static void main(String[] args) {
        double[] qd = {0.5, 0.0, -0.2};
        double[][] J = {
            {1.0, 0.2, 0.0},
            {0.0, 1.0, 0.3}
        };
        double[] yd = new double[2];
        for(int r=0; r<2; r++){
            for(int c=0; c<3; c++){
                yd[r] += J[r][c]*qd[c];
            }
        }
        System.out.println("yd = " + Arrays.toString(yd));

        // Typical Java robotics ecosystems:
        // - rosjava (ROS1-era), PX4 Java MAVLink tools, Android-based robots.
    }
}
      