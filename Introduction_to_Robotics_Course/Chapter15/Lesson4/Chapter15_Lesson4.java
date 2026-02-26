public class SafetyMonitor implements Runnable {

    private final RobotInterface robot;
    private final HumanTracker tracker;
    private static final double SAFE_DISTANCE = 0.8;
    private static final double SPEED_LIMIT   = 0.25;

    public SafetyMonitor(RobotInterface robot, HumanTracker tracker) {
        this.robot = robot;
        this.tracker = tracker;
    }

    @Override
    public void run() {
        while (true) {
            double speed = robot.getEndEffectorSpeed();
            double distance = tracker.getMinHumanDistance();

            if (distance <= SAFE_DISTANCE && speed > SPEED_LIMIT) {
                System.out.println("Safety violation detected, stopping robot.");
                robot.commandEmergencyStop();
            }

            try {
                Thread.sleep(10); // 100 Hz
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }
}
      
