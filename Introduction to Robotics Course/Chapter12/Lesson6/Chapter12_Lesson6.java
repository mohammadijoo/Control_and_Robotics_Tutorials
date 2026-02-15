// ROS2 Java (rcljava) seeded randomness
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.SingleThreadedExecutor;
import std_msgs.msg.Float32;
import java.util.Random;

public class NoisyPublisher extends Node {
  private final Random rng;
  public NoisyPublisher() {
    super("noisy_pub");
    this.declareParameter("seed", 0);
    int seed = this.getParameter("seed").asInteger();
    rng = new Random(seed);

    var pub = this.<Float32>createPublisher(Float32.class, "noise");
    this.createWallTimer(100, () -> {
      float z = (float)(rng.nextGaussian());
      Float32 msg = new Float32();
      msg.setData(z);
      pub.publish(msg);
    });
  }
  public static void main(String[] args) {
    RCLJava.rclJavaInit();
    var exec = new SingleThreadedExecutor();
    exec.addNode(new NoisyPublisher());
    exec.spin();
  }
}
      
