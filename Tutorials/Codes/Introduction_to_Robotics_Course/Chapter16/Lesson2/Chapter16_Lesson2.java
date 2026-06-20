import java.util.ArrayList;
import java.util.List;

class RobotConcept {
    String name;
    double cost;
    double mass;
    double energy;
    double trackingError;

    RobotConcept(String name, double cost, double mass, double energy, double trackingError) {
        this.name = name;
        this.cost = cost;
        this.mass = mass;
        this.energy = energy;
        this.trackingError = trackingError;
    }
}

public class ConceptTradeoff {
    private static double norm(double x, double xmin, double xmax) {
        if (Math.abs(xmax - xmin) < 1e-9) return 1.0;
        return (xmax - x) / (xmax - xmin);
    }

    public static void main(String[] args) {
        List<RobotConcept> concepts = new ArrayList<>();
        concepts.add(new RobotConcept("A_wheeled", 15.0, 40.0, 800.0, 1.5));
        concepts.add(new RobotConcept("B_tracked", 18.0, 50.0, 700.0, 1.2));
        concepts.add(new RobotConcept("C_legged", 25.0, 55.0, 950.0, 0.8));

        final double MAX_MASS = 55.0;
        final double MAX_ENERGY = 1000.0;

        List<RobotConcept> feasible = new ArrayList<>();
        for (RobotConcept c : concepts) {
            if (c.mass <= MAX_MASS && c.energy <= MAX_ENERGY) {
                feasible.add(c);
            }
        }

        double costMin = Double.POSITIVE_INFINITY, costMax = Double.NEGATIVE_INFINITY;
        double massMin = Double.POSITIVE_INFINITY, massMax = Double.NEGATIVE_INFINITY;
        double energyMin = Double.POSITIVE_INFINITY, energyMax = Double.NEGATIVE_INFINITY;
        double errorMin = Double.POSITIVE_INFINITY, errorMax = Double.NEGATIVE_INFINITY;

        for (RobotConcept c : feasible) {
            costMin = Math.min(costMin, c.cost);
            costMax = Math.max(costMax, c.cost);
            massMin = Math.min(massMin, c.mass);
            massMax = Math.max(massMax, c.mass);
            energyMin = Math.min(energyMin, c.energy);
            energyMax = Math.max(energyMax, c.energy);
            errorMin = Math.min(errorMin, c.trackingError);
            errorMax = Math.max(errorMax, c.trackingError);
        }

        double wCost = 0.3, wMass = 0.2, wEnergy = 0.2, wError = 0.3;

        double bestU = -1e9;
        String bestName = "";

        for (RobotConcept c : feasible) {
            double zCost = norm(c.cost, costMin, costMax);
            double zMass = norm(c.mass, massMin, massMax);
            double zEnergy = norm(c.energy, energyMin, energyMax);
            double zError = norm(c.trackingError, errorMin, errorMax);

            double U = wCost * zCost + wMass * zMass + wEnergy * zEnergy + wError * zError;
            System.out.printf("%s : U = %.3f%n", c.name, U);

            if (U > bestU) {
                bestU = U;
                bestName = c.name;
            }
        }

        System.out.println("Best concept: " + bestName + " with U = " + bestU);
    }
}
      
