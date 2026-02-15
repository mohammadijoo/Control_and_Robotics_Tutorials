import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

class Config {
    public double Kp = 4.0;
    public double Ki = 3.0;
    public double dt = 0.001;
    public double tFinal = 10.0;
    public double settlingBand = 0.02;
}

// Same plant as before: G(s) = 1 / (s + 1)^2, PI controller
class PlantPI {
    private Config cfg;
    private double x1 = 0.0;
    private double x2 = 0.0;
    private double xI = 0.0;

    public PlantPI(Config cfg) {
        this.cfg = cfg;
    }

    public void reset() {
        x1 = 0.0;
        x2 = 0.0;
        xI = 0.0;
    }

    public double step(double r) {
        double y = x1;
        double e = r - y;
        xI += e * cfg.dt;
        double u = cfg.Kp * e + cfg.Ki * xI;

        double x1dot = -x1 + x2;
        double x2dot = -x2 + u;

        x1 += x1dot * cfg.dt;
        x2 += x2dot * cfg.dt;
        return y;
    }
}

public class PiExperiment {
    public static void main(String[] args) throws IOException {
        Config cfg = new Config();
        PlantPI system = new PlantPI(cfg);

        int nSteps = (int) Math.round(cfg.tFinal / cfg.dt);
        List<Double> tList = new ArrayList<>(nSteps + 1);
        List<Double> yList = new ArrayList<>(nSteps + 1);

        double t = 0.0;
        for (int k = 0; k <= nSteps; ++k) {
            double y = system.step(1.0);
            tList.add(t);
            yList.add(y);
            t += cfg.dt;
        }

        // Metrics
        double yFinal = yList.get(yList.size() - 1);
        double Mp = 0.0;
        for (double y : yList) {
            double val = y - 1.0;
            if (val > Mp) Mp = val;
        }
        double ess = 1.0 - yFinal;

        double ts = 0.0;
        for (int k = nSteps; k >= 0; --k) {
            double y = yList.get(k);
            if (Math.abs(y - 1.0) > cfg.settlingBand) {
                ts = tList.get(k);
                break;
            }
        }

        // CSV export
        try (FileWriter fw = new FileWriter("java_pi_closed_loop.csv")) {
            fw.write("t,y\n");
            for (int k = 0; k < tList.size(); ++k) {
                fw.write(tList.get(k) + "," + yList.get(k) + "\n");
            }
        }

        // Metrics export
        try (FileWriter fw = new FileWriter("java_pi_metrics.txt")) {
            fw.write("Kp=" + cfg.Kp + "\n");
            fw.write("Ki=" + cfg.Ki + "\n");
            fw.write("Mp=" + Mp + "\n");
            fw.write("ess=" + ess + "\n");
            fw.write("ts=" + ts + "\n");
        }

        // In robot control libraries such as WPILib, the same numerical
        // experiment can be run in a test or simulation mode, reading cfg
        // from a configuration file and writing back metrics for documentation.
    }
}
