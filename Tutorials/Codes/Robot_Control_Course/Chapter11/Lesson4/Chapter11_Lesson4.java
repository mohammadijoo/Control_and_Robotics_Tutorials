
public class JointComplementaryFilter {
    private double Ts;      // sample time
    private double alpha;   // fusion weight
    private double qHat;    // filtered angle

    public JointComplementaryFilter(double Ts, double alpha, double q0) {
        this.Ts = Ts;
        this.alpha = alpha;
        this.qHat = q0;
    }

    public double update(double omegaImu, double qEnc) {
        // prediction (integrate IMU)
        double qPred = qHat + Ts * omegaImu;
        // fusion with encoder
        qHat = alpha * qPred + (1.0 - alpha) * qEnc;
        return qHat;
    }

    public static void main(String[] args) {
        double Ts = 0.001;
        double alpha = 0.95;
        JointComplementaryFilter filter =
                new JointComplementaryFilter(Ts, alpha, 0.0);

        double qEnc = 0.0;
        double omegaImu = 0.0;

        for (int k = 0; k < 10000; ++k) {
            // here we would read encoder and IMU from hardware
            // qEnc = readEncoder();
            // omegaImu = readImuGyro();

            double qFiltered = filter.update(omegaImu, qEnc);
            // controller would use qFiltered instead of raw sensor values
        }
    }
}
