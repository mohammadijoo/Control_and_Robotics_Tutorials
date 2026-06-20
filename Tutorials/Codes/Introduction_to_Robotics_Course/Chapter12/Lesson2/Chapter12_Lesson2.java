import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;
import std_msgs.msg.Float64;

public class TopicDemo {
  public static void main(String[] args) {
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("java_topic_demo");

    Publisher<Float64> pub =
      node.createPublisher(Float64.class, "/u_cmd");

    Subscription<Float64> sub =
      node.createSubscription(Float64.class, "/u_cmd",
        msg -> System.out.println("u_cmd=" + msg.getData()));

    node.createWallTimer(10, () -> {
      Float64 m = new Float64();
      m.setData(1.0);
      pub.publish(m);
    });

    RCLJava.spin(node);
  }
}
      
