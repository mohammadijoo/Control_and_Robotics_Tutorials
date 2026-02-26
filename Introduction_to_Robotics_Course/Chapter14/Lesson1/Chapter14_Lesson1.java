import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import geometry_msgs.Twist;

public class GuiTeleopNode extends AbstractNodeMain {

    private Publisher<Twist> cmdPub;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("gui_teleop_node");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        cmdPub = connectedNode.newPublisher("/cmd_vel", Twist._TYPE);
    }

    // Example handler for a key press (e.g. from a Swing GUI)
    public void handleKeyCommand(String key) {
        Twist twist = cmdPub.newMessage();
        if ("W".equals(key)) {
            twist.getLinear().setX(0.3);
        } else if ("S".equals(key)) {
            twist.getLinear().setX(-0.3);
        } else {
            twist.getLinear().setX(0.0);
        }
        twist.getAngular().setZ(0.0);
        cmdPub.publish(twist);
    }
}
      
