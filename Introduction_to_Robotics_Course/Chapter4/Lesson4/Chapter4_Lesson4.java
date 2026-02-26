
import java.util.*;

class Module {
    public String name;
    public double reliability; // R_i
    public Module(String name, double r) {
        this.name = name; this.reliability = r;
    }
}

public class ReliabilityDemo {

    // Series reliability: product of R_i
    static double series(List<Module> mods) {
        double R = 1.0;
        for (Module m : mods) R *= m.reliability;
        return R;
    }

    // Parallel reliability: 1 - product(1 - R_j)
    static double parallel(List<Module> mods) {
        double failProd = 1.0;
        for (Module m : mods) failProd *= (1.0 - m.reliability);
        return 1.0 - failProd;
    }

    public static void main(String[] args) {
        List<Module> sensorTriplet = Arrays.asList(
            new Module("IMU1", 0.96),
            new Module("IMU2", 0.96),
            new Module("IMU3", 0.96)
        );

        System.out.println("Series R (all needed): " + series(sensorTriplet));
        System.out.println("Parallel R (any works): " + parallel(sensorTriplet));
    }
}
      