import java.util.logging.Logger;

public class DebugController {
    private static final Logger log = Logger.getLogger("DebugController");
    private double ref = 0.0;
    private double y = 0.0;
    private double gamma = 0.5;
    private long lastTimeNs = System.nanoTime();

    public void periodic() {
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) * 1e-9;
        lastTimeNs = now;

        if (dt > 0.05) {
            log.warning(String.format("Loop overrun: dt=%.3f s", dt));
        }

        double e = ref - y;
        if (Math.abs(e) > gamma) {
            log.warning(String.format("Tracking error too large: e=%.3f", e));
        }
    }

    public void setReference(double r) { this.ref = r; }
    public void setMeasurement(double y) { this.y = y; }
}
      
