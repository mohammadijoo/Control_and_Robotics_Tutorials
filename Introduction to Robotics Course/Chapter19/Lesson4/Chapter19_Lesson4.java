public interface RangeSensors {
    double getFront();
    Double getRight(); // may be null if not available
}

public interface MobileBase {
    void setVelocity(double v, double omega);
}

public class RuleBasedAutonomy implements Runnable {

    private final RangeSensors sensors;
    private final MobileBase base;
    private final double dFrontSafe;
    private final double dRightRef;
    private final double vCruise;
    private final double omegaTurn;
    private final double kOmega;
    private final long periodMillis;

    public RuleBasedAutonomy(RangeSensors sensors,
                             MobileBase base,
                             long periodMillis) {
        this.sensors = sensors;
        this.base = base;
        this.periodMillis = periodMillis;
        this.dFrontSafe = 0.5;
        this.dRightRef = 0.6;
        this.vCruise = 0.2;
        this.omegaTurn = 0.6;
        this.kOmega = 1.0;
    }

    @Override
    public void run() {
        while (true) {
            double dFront = sensors.getFront();
            Double dRightObj = sensors.getRight();
            boolean haveRight = (dRightObj != null);
            double v = 0.0;
            double omega = 0.0;

            if (dFront < dFrontSafe) {
                v = 0.0;
                omega = omegaTurn;
            } else if (haveRight) {
                double dRight = dRightObj.doubleValue();
                v = vCruise;
                double errorRight = dRight - dRightRef;
                omega = kOmega * errorRight;
            } else {
                v = vCruise;
                omega = 0.0;
            }

            base.setVelocity(v, omega);

            try {
                Thread.sleep(periodMillis);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }
}
      
