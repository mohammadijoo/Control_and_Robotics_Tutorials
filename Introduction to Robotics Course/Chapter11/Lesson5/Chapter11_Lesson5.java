import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

public class HeartbeatMonitor {
    private final long timeoutMs;
    private final AtomicLong lastHbMs = new AtomicLong(System.currentTimeMillis());
    private final ScheduledExecutorService exec = Executors.newScheduledThreadPool(1);

    public HeartbeatMonitor(long timeoutMs) {
        this.timeoutMs = timeoutMs;
    }

    public void heartbeat() {
        lastHbMs.set(System.currentTimeMillis());
    }

    public void start() {
        exec.scheduleAtFixedRate(() -> {
            long dt = System.currentTimeMillis() - lastHbMs.get();
            if (dt > timeoutMs) {
                System.out.println("[WATCHDOG] heartbeat late; safe mode.");
                exec.shutdown();
            }
        }, 0, timeoutMs / 4, TimeUnit.MILLISECONDS);
    }
}
      
