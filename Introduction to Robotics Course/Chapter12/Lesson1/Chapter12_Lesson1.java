// PublisherJava.java (conceptual skeleton)
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Float64;

public class PublisherJava extends AbstractNodeMain {
  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final Publisher<Float64> pub =
        connectedNode.newPublisher("sine", Float64._TYPE);

    connectedNode.executeCancellableLoop(new org.ros.concurrent.CancellableLoop() {
      private int k = 0;
      @Override
      protected void loop() throws InterruptedException {
        Float64 msg = pub.newMessage();
        msg.setData(Math.sin(0.1 * k));
        pub.publish(msg);
        k++;
        Thread.sleep(10); // 100 Hz
      }
    });
  }

  @Override
  public org.ros.namespace.GraphName getDefaultNodeName() {
    return org.ros.namespace.GraphName.of("sine_publisher_java");
  }
}
      
