
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class JitterExample {
    public static void main(String[] args) {
        ScheduledExecutorService exec = Executors.newSingleThreadScheduledExecutor();

        final long periodNs = 1_000_000; // 1 ms
        final long[] expectedNext = { System.nanoTime() + periodNs };

        Runnable task = () -> {
            long now = System.nanoTime();
            long jitter = now - expectedNext[0];
            System.out.println("jitter (ns) = " + jitter);

            // TODO: read sensors, compute control, send commands

            expectedNext[0] += periodNs;
        };

        exec.scheduleAtFixedRate(task, 0, 1, TimeUnit.MILLISECONDS);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        exec.shutdown();
    }
}
