// Chapter14_Lesson1.java
// ROS 2 (rcljava) layered navigation safety-filter skeleton.
// Note: rcljava setup depends on your ROS 2 distribution and build tooling.
//
// Author: Abolfazl Mohammadijoo (course material)
// License: MIT (see repository)

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.executors.SingleThreadedExecutor;

import geometry_msgs.msg.Twist;
import nav_msgs.msg.Odometry;
import sensor_msgs.msg.LaserScan;

public class Chapter14_Lesson1 {

  // Small internal state
  private static class Pose2D {
    double x = 0.0, y = 0.0, yaw = 0.0;
  }

  private static double wrapToPi(double a) {
    while (a > Math.PI) a -= 2.0 * Math.PI;
    while (a < -Math.PI) a += 2.0 * Math.PI;
    return a;
  }

  private static double yawFromQuat(double w, double x, double y, double z) {
    double siny = 2.0 * (w * z + x * y);
    double cosy = 1.0 - 2.0 * (y * y + z * z);
    return Math.atan2(siny, cosy);
  }

  private static double minFiniteRange(float[] ranges) {
    double mr = Double.POSITIVE_INFINITY;
    for (float r : ranges) {
      if (Float.isFinite(r) && r > 0.0f) {
        mr = Math.min(mr, (double) r);
      }
    }
    return mr;
  }

  private static double[] safetyFilterScan(double v, double w, double minRange, double dStop, double dSlow) {
    if (minRange <= dStop) {
      return new double[] {0.0, 0.0};
    }
    if (minRange <= dSlow) {
      double s = (minRange - dStop) / (dSlow - dStop + 1e-9);
      s = Math.max(0.0, Math.min(1.0, s));
      return new double[] {s * v, s * w};
    }
    return new double[] {v, w};
  }

  public static void main(String[] args) {
    RCLJava.rclJavaInit();

    final Node node = RCLJava.createNode("layered_nav_filter_java");

    final Pose2D pose = new Pose2D();
    final double[] minRange = new double[] {Double.POSITIVE_INFINITY};

    // Parameters (could be declared dynamically; kept simple here)
    final double vRef  = 0.4;
    final double kYaw  = 1.8;
    final double dStop = 0.35;
    final double dSlow = 0.9;

    // Subscriptions
    node.<Odometry>createSubscription(Odometry.class, "/odom", msg -> {
      pose.x = msg.getPose().getPose().getPosition().getX();
      pose.y = msg.getPose().getPose().getPosition().getY();
      var q = msg.getPose().getPose().getOrientation();
      pose.yaw = yawFromQuat(q.getW(), q.getX(), q.getY(), q.getZ());
    });

    node.<LaserScan>createSubscription(LaserScan.class, "/scan", msg -> {
      minRange[0] = minFiniteRange(msg.getRanges());
    });

    // Publisher
    final var pubCmd = node.<Twist>createPublisher(Twist.class, "/cmd_vel");

    // Timer at 20 Hz (local layer); reactive filter applied each tick
    node.createWallTimer(50, () -> {
      // Very simple nominal local policy:
      double v = vRef;
      double w = -kYaw * wrapToPi(pose.yaw - 0.0);

      double[] u = safetyFilterScan(v, w, minRange[0], dStop, dSlow);

      Twist cmd = new Twist();
      cmd.getLinear().setX(u[0]);
      cmd.getAngular().setZ(u[1]);
      pubCmd.publish(cmd);
    });

    SingleThreadedExecutor exec = new SingleThreadedExecutor();
    exec.addNode(node);
    exec.spin();

    node.dispose();
    RCLJava.shutdown();
  }
}
