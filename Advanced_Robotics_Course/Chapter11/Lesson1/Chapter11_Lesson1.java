import java.util.ArrayList;
import java.util.List;

public class Demonstration {
    public static class Sample {
        public double t;
        public double[] x; // state
        public double[] u; // control (optional)

        public Sample(double t, double[] x, double[] u) {
            this.t = t;
            this.x = x;
            this.u = u;
        }
    }

    private final List<Sample> samples = new ArrayList<>();

    public void addSample(double t, double[] x, double[] u) {
        samples.add(new Sample(t, x, u));
    }

    public List<Sample> getSamples() {
        return samples;
    }
}

// Example usage: teleop application calling into the logger
public class TeleopApp {
    private final Demonstration demo = new Demonstration();
    private double startTime;

    public void onStart() {
        startTime = System.currentTimeMillis() / 1000.0;
    }

    // This would be called periodically with robot state and teleop input
    public void onUpdate(double[] stateX, double[] teleopU) {
        double t = System.currentTimeMillis() / 1000.0 - startTime;
        demo.addSample(t, stateX, teleopU);
    }

    public Demonstration getDemo() {
        return demo;
    }
}
      
