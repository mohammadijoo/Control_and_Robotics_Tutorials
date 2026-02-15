public class SdrChecker {
    private final double mpMax;
    private final double tsMax;

    public SdrChecker(double mpMax, double tsMax) {
        this.mpMax = mpMax;
        this.tsMax = tsMax;
    }

    public boolean check(double mpMeasured, double tsMeasured) {
        return mpMeasured <= mpMax && tsMeasured <= tsMax;
    }

    public static void main(String[] args) {
        SdrChecker checker = new SdrChecker(0.10, 1.5);

        // Example: results from an experiment or simulation
        double mp = 0.08;
        double ts = 1.2;

        if (checker.check(mp, ts)) {
            System.out.println("SDR: requirements satisfied.");
        } else {
            System.out.println("SDR: requirements NOT satisfied.");
        }
    }
}
      
