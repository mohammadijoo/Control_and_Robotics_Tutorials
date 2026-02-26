public class PrototypeTester {

    public static class RunResult {
        public final double settlingTime;
        public final boolean pass;
        public RunResult(double Ts, boolean pass) {
            this.settlingTime = Ts;
            this.pass = pass;
        }
    }

    // Stub: would call into ROS, gRPC, or a custom protocol
    private static RunResult executeTest(double Kp) {
        // Send configuration to robot, start a step test, wait for log
        double TsMeasured = 1.2; // placeholder
        boolean pass = TsMeasured <= 1.5;
        return new RunResult(TsMeasured, pass);
    }

    public static void main(String[] args) {
        double[] KpValues = {0.5, 1.0, 2.0};
        int N = 10; // repetitions per gain

        for (double Kp : KpValues) {
            double sumTs = 0.0;
            int passes = 0;
            for (int i = 0; i < N; i++) {
                RunResult r = executeTest(Kp);
                sumTs += r.settlingTime;
                if (r.pass) {
                    passes++;
                }
            }
            double meanTs = sumTs / N;
            double pHat = (double) passes / N;
            System.out.printf("Kp=%.2f, mean Ts=%.3fs, pass rate=%.2f%n",
                              Kp, meanTs, pHat);
        }
    }
}
      
