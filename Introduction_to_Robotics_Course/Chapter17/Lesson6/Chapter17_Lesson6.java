public class DeploymentMetrics {

    public static class Metrics {
        public double lambdaHat;
        public double mtbfHat;
        public double availabilityHat;
        public double missionSuccessProb;
    }

    public static Metrics compute(double totalUptimeHours,
                                  int nFailures,
                                  double mttrHours,
                                  double missionHours) {
        if (totalUptimeHours <= 0.0) {
            throw new IllegalArgumentException("totalUptimeHours must be positive");
        }
        if (nFailures <= 0) {
            nFailures = 1; // conservative
        }

        double lambdaHat = nFailures / totalUptimeHours;
        double mtbfHat = 1.0 / lambdaHat;
        double availabilityHat = mtbfHat / (mtbfHat + mttrHours);
        double missionSuccess = Math.exp(-lambdaHat * missionHours);

        Metrics m = new Metrics();
        m.lambdaHat = lambdaHat;
        m.mtbfHat = mtbfHat;
        m.availabilityHat = availabilityHat;
        m.missionSuccessProb = missionSuccess;
        return m;
    }

    public static void main(String[] args) {
        Metrics m = compute(10000.0, 5, 2.0, 8.0);
        System.out.println("lambda_hat: " + m.lambdaHat);
        System.out.println("MTBF_hat: " + m.mtbfHat);
        System.out.println("availability_hat: " + m.availabilityHat);
        System.out.println("mission_success_prob: " + m.missionSuccessProb);
    }
}
      
