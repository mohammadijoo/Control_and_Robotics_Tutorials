import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.Socket;
import java.util.concurrent.ConcurrentLinkedQueue;

public class StreamConsumer {
    private final ConcurrentLinkedQueue<Sample> buffer =
            new ConcurrentLinkedQueue<>();

    private static class Sample {
        final double t;
        final double value;
        Sample(double t, double value) {
            this.t = t;
            this.value = value;
        }
    }

    public void startReader(String host, int port) throws Exception {
        Socket socket = new Socket(host, port);
        BufferedReader br = new BufferedReader(
                new InputStreamReader(socket.getInputStream()));

        new Thread(() -> {
            String line;
            long t0 = System.nanoTime();
            try {
                while ((line = br.readLine()) != null) {
                    double val = Double.parseDouble(line.trim());
                    double t = 1e-9 * (System.nanoTime() - t0);
                    buffer.add(new Sample(t, val));
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }, "stream-reader").start();
    }

    public ConcurrentLinkedQueue<Sample> getBuffer() {
        return buffer;
    }

    // GUI code (e.g. JavaFX) would periodically read from buffer and update plots.
}
      
