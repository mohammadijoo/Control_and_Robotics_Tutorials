import java.util.logging.*;
import java.io.IOException;
import java.util.ArrayDeque;

public class RobotMonitor {
    static Logger logger = Logger.getLogger("robot");
    static ArrayDeque<Double> window = new ArrayDeque<>();
    static int W = 50;

    public static void setup() throws IOException {
        FileHandler fh = new FileHandler("robot_run_java.log");
        fh.setFormatter(new SimpleFormatter());
        logger.addHandler(fh);
        logger.setLevel(Level.INFO);
    }

    public static void step(double u, double y, double cpu) {
        logger.info(String.format("u=%.3f, y=%.3f, cpu=%.2f", u, y, cpu));

        window.add(cpu);
        if(window.size() > W) window.poll();
        double avg = 0.0;
        for(double c : window) avg += c;
        avg /= window.size();

        if(avg > 0.9) {
            logger.warning("High CPU average: " + avg);
        }
    }

    public static void main(String[] args) throws IOException {
        setup();
        for(int k=0; k<100; k++){
            step(0.1, 0.0, 0.5);
        }
    }
}
      
