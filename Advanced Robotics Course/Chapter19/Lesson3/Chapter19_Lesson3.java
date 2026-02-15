import java.util.ArrayDeque;
import java.util.Deque;

class Pose2D {
    public double x;
    public double y;
    public double theta;

    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}

class PoseMemory {
    private final int windowSize;
    private final Deque<Pose2D> buffer;

    public PoseMemory(int windowSize) {
        this.windowSize = windowSize;
        this.buffer = new ArrayDeque<>(windowSize);
    }

    public void addPose(Pose2D pose) {
        if (buffer.size() == windowSize) {
            buffer.removeFirst();
        }
        buffer.addLast(pose);
    }

    public Pose2D getMostRecent() {
        return buffer.peekLast();
    }

    public Pose2D getOldest() {
        return buffer.peekFirst();
    }

    public int size() {
        return buffer.size();
    }
}

class LongHorizonNavigator {
    private final PoseMemory memory;
    private final double goalX;
    private final double goalY;

    public LongHorizonNavigator(int windowSize, double goalX, double goalY) {
        this.memory = new PoseMemory(windowSize);
        this.goalX = goalX;
        this.goalY = goalY;
    }

    public void update(Pose2D currentPose) {
        memory.addPose(currentPose);
    }

    // Simple long-horizon progress metric using oldest vs newest pose
    public double computeProgress() {
        if (memory.size() < 2) {
            return 0.0;
        }
        Pose2D oldest = memory.getOldest();
        Pose2D newest = memory.getMostRecent();
        double dOld = distance(oldest.x, oldest.y, goalX, goalY);
        double dNew = distance(newest.x, newest.y, goalX, goalY);
        return dOld - dNew; // positive if we moved closer
    }

    private static double distance(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return Math.sqrt(dx * dx + dy * dy);
    }
}

public class LongHorizonDemo {
    public static void main(String[] args) {
        LongHorizonNavigator nav = new LongHorizonNavigator(50, 10.0, 0.0);
        for (int t = 0; t < 100; ++t) {
            // Fake pose updates along x-axis
            Pose2D pose = new Pose2D(0.1 * t, 0.0, 0.0);
            nav.update(pose);
            double progress = nav.computeProgress();
            System.out.println("t=" + t + " progress=" + progress);
        }
    }
}
      
