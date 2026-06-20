import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import geometry_msgs.msg.TransformStamped;
import tf2_msgs.msg.TFMessage;
// Note: exact TF broadcaster helpers may vary by distro;
// this is a minimal "usage" skeleton.

public class TFJavaBroadcaster {
  public static void main(String[] args){
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("java_tf_broadcaster");

    var pub = node.<TFMessage>createPublisher(TFMessage.class, "/tf");

    node.createWallTimer(33, () -> {
      TransformStamped t = new TransformStamped();
      t.getHeader().setFrameId("base_link");
      t.setChildFrameId("gps_frame");
      t.getHeader().getStamp().setSec((int)(System.currentTimeMillis()/1000));

      t.getTransform().getTranslation().setX(0.0);
      t.getTransform().getTranslation().setY(-0.2);
      t.getTransform().getTranslation().setZ(0.3);

      t.getTransform().getRotation().setW(1.0); // identity rotation

      TFMessage msg = new TFMessage();
      msg.setTransforms(java.util.Arrays.asList(t));
      pub.publish(msg);
    });

    RCLJava.spin(node);
    node.dispose();
    RCLJava.shutdown();
  }
}
      
