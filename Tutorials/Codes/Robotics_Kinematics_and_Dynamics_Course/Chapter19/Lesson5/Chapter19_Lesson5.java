import org.ejml.simple.SimpleMatrix;

public class SingleLinkIdentification {

    public static void main(String[] args) {
        double Ts = 0.002;
        double Tend = 8.0;
        int N = (int) (Tend / Ts);
        double[] t = new double[N];

        for (int k = 0; k < N; ++k) {
            t[k] = k * Ts;
        }

        double[] q = new double[N];
        double[] dq = new double[N];
        double[] ddq = new double[N];
        double piVal = Math.PI;

        for (int k = 0; k < N; ++k) {
            double tk = t[k];
            q[k] = 0.6 * Math.sin(2.0 * piVal * 0.4 * tk)
                 + 0.4 * Math.sin(2.0 * piVal * 0.9 * tk);
            dq[k] = 0.6 * 2.0 * piVal * 0.4 * Math.cos(2.0 * piVal * 0.4 * tk)
                  + 0.4 * 2.0 * piVal * 0.9 * Math.cos(2.0 * piVal * 0.9 * tk);
            ddq[k] = -0.6 * Math.pow(2.0 * piVal * 0.4, 2) * Math.sin(2.0 * piVal * 0.4 * tk)
                    -0.4 * Math.pow(2.0 * piVal * 0.9, 2) * Math.sin(2.0 * piVal * 0.9 * tk);
        }

        // Build Y and tau
        SimpleMatrix Y = new SimpleMatrix(N, 3);
        SimpleMatrix tau = new SimpleMatrix(N, 1);
        double[] piTrue = {0.35, 2.1, 0.08};

        for (int k = 0; k < N; ++k) {
            double row0 = ddq[k];
            double row1 = Math.cos(q[k]);
            double row2 = dq[k];
            Y.set(k, 0, row0);
            Y.set(k, 1, row1);
            Y.set(k, 2, row2);
            double tauVal = row0 * piTrue[0] + row1 * piTrue[1] + row2 * piTrue[2];
            tau.set(k, 0, tauVal);
        }

        // pi_hat = (Y^T Y)^(-1) Y^T tau
        SimpleMatrix YTY = Y.transpose().mult(Y);
        SimpleMatrix YTtau = Y.transpose().mult(tau);
        SimpleMatrix piHat = YTY.invert().mult(YTtau);

        System.out.println("Estimated parameters:");
        piHat.print();
    }
}
      
