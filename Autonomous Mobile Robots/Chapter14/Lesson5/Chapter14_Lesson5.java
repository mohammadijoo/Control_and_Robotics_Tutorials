/*
Chapter 14 - Lesson 5: Lab: Assemble a Full Navigation Stack

Java ROS 2 example (rcljava) - NavigateToPose action client skeleton.

Notes:
- ROS 2 Java (rcljava) setup varies by ROS distribution and build method.
- Action support may require additional packages depending on your setup.
- This file is written as a clear template for students to adapt.

Conceptual steps:
1) Initialize RCLJava
2) Create a node
3) Create an action client for nav2_msgs/action/NavigateToPose
4) Send goal and handle feedback + result
*/

package amr.chapter14.lesson5;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;

import geometry_msgs.msg.PoseStamped;
import nav2_msgs.action.NavigateToPose;

public class Chapter14Lesson5 {
  private static PoseStamped makeGoalPose(Node node, double x, double y, double yawRad, String frameId) {
    PoseStamped pose = new PoseStamped();
    pose.getHeader().setFrameId(frameId);
    pose.getHeader().setStamp(node.getClock().now().toMsg());

    pose.getPose().getPosition().setX(x);
    pose.getPose().getPosition().setY(y);
    pose.getPose().getPosition().setZ(0.0);

    pose.getPose().getOrientation().setX(0.0);
    pose.getPose().getOrientation().setY(0.0);
    pose.getPose().getOrientation().setZ(Math.sin(yawRad / 2.0));
    pose.getPose().getOrientation().setW(Math.cos(yawRad / 2.0));
    return pose;
  }

  public static void main(String[] args) throws Exception {
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("chapter14_lesson5_nav_client_java");

    // PSEUDOCODE (API names can differ per rcljava version):
    // ActionClient<NavigateToPose.Goal, NavigateToPose.Result, NavigateToPose.Feedback> client =
    //     new ActionClient<>(node, NavigateToPose.class, "navigate_to_pose");
    //
    // if (!client.waitForServer(2_000)) { ... }
    //
    // NavigateToPose.Goal goal = new NavigateToPose.Goal();
    // goal.setPose(makeGoalPose(node, 2.0, 0.5, 0.0, "map"));
    //
    // client.sendGoal(goal,
    //   (feedback) -> node.getLogger().info("distance_remaining=" + feedback.getDistanceRemaining()),
    //   (result) -> { node.getLogger().info("DONE: " + result); RCLJava.shutdown(); }
    // );

    NavigateToPose.Goal goal = new NavigateToPose.Goal();
    goal.setPose(makeGoalPose(node, 2.0, 0.5, 0.0, "map"));

    node.getLogger().info("Template prepared. Implement ActionClient calls for your rcljava version.");

    for (int i = 0; i < 20; i++) {
      RCLJava.spinOnce(node, 100);
      Thread.sleep(100);
    }

    node.dispose();
    RCLJava.shutdown();
  }
}
