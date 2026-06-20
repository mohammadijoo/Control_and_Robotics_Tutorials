public class Hazard {
    public final String name;
    public final int severity;
    public final int probability;

    public Hazard(String name, int severity, int probability) {
        this.name = name;
        this.severity = severity;
        this.probability = probability;
    }

    public int riskIndex() {
        return severity * probability;
    }

    public String riskLevel() {
        int I = riskIndex();
        if (I <= 4) return "low";
        if (I <= 8) return "medium";
        return "high";
    }

    public static void main(String[] args) {
        Hazard[] hazards = new Hazard[] {
                new Hazard("Pinch at wrist joint", 3, 3),
                new Hazard("Controller overtemperature", 2, 2),
                new Hazard("Unexpected fast motion", 4, 3)
        };
        for (Hazard h : hazards) {
            System.out.println(h.name + ": I=" + h.riskIndex()
                    + ", level=" + h.riskLevel());
        }
    }
}
      
