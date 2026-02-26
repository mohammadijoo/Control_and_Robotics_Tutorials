import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;
import std_msgs.msg.String;

public class SimplePublisherJava {
  public static void main(String[] args) {
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("simple_publisher_java");
    Publisher<String> pub = node.createPublisher(String.class, "chatter");

    int count = 0;
    while (RCLJava.ok()) {
      String msg = new String();
      msg.setData("hello " + count++);
      pub.publish(msg);
      try { Thread.sleep(500); } catch (InterruptedException e) {}
    }
    RCLJava.shutdown();
  }
}
      
