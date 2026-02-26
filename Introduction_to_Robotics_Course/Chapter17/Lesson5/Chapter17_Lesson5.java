public class NDVIMonitor {

    private double threshold;
    private int consecutiveLimit;
    private int consecutiveStressed;

    public NDVIMonitor(double threshold, int consecutiveLimit) {
        this.threshold = threshold;
        this.consecutiveLimit = consecutiveLimit;
        this.consecutiveStressed = 0;
    }

    public void handleMeasurement(double ndvi, double x, double y) {
        // Update internal counter based on NDVI threshold
        if (ndvi < threshold) {
            consecutiveStressed++;
        } else {
            consecutiveStressed = 0;
        }

        if (consecutiveStressed >= consecutiveLimit) {
            triggerAlert(x, y, ndvi);
            // Reset after alert
            consecutiveStressed = 0;
        }
    }

    private void triggerAlert(double x, double y, double ndvi) {
        System.out.println("Stress alert at (" + x + ", " + y + "), NDVI=" + ndvi);
        // In a real system, send message to actuator or farm management software
    }

    public static void main(String[] args) {
        NDVIMonitor monitor = new NDVIMonitor(0.3, 5);

        // Example test data (e.g., along a robot path)
        double[] ndviSamples = {0.4, 0.35, 0.28, 0.25, 0.27, 0.26, 0.45};
        double x = 0.0, y = 0.0;
        double dx = 1.0;

        for (double ndvi : ndviSamples) {
            monitor.handleMeasurement(ndvi, x, y);
            x += dx;
        }
    }
}
      
