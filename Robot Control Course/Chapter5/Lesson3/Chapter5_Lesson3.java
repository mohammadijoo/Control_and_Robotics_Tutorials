
public class SaturatedPDController {
    private final double[] Kp;
    private final double[] Kd;
    private final double[] tauMin;
    private final double[] tauMax;

    public SaturatedPDController(double[] Kp, double[] Kd,
                                 double[] tauMin, double[] tauMax) {
        this.Kp = Kp.clone();
        this.Kd = Kd.clone();
        this.tauMin = tauMin.clone();
        this.tauMax = tauMax.clone();
    }

    public void computeTau(double[] q, double[] qd,
                           double[] qRef, double[] qdRef,
                           double[] tauOut) {
        int n = q.length;
        for (int i = 0; i < n; i++) {
            double e  = q[i]  - qRef[i];
            double ed = qd[i] - qdRef[i];
            double tauNom = -Kp[i] * e - Kd[i] * ed;

            // Saturation
            double tau = tauNom;
            if (tau > tauMax[i]) tau = tauMax[i];
            if (tau < tauMin[i]) tau = tauMin[i];
            tauOut[i] = tau;
        }
    }
}
