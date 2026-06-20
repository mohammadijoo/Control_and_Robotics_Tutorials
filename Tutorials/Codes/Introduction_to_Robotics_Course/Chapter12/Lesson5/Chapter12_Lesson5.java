// controller_pkg/src/main/java/ControllerNode.java
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;
import std_msgs.msg.Float64;

public class ControllerNode {
  public static void main(String[] args) {
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("controller_node");

    final double Kp = 1.5;
    final double r = 1.0;

    Publisher<Float64> pub = node.<Float64>createPublisher(
        Float64.class, "/cmd");

    Subscription<Float64> sub = node.<Float64>createSubscription(
        Float64.class, "/sensor",
        msg -> {
          double y = msg.getData();
          Float64 out = new Float64();
          out.setData(Kp * (r - y));
          pub.publish(out);
        });

    RCLJava.spin(node);
    node.dispose();
    RCLJava.shutdown();
  }
}
