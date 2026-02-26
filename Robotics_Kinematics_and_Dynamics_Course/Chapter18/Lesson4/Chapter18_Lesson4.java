public class TimeParameterizedPath {

    // q_p(s)
    static double[] qPath(double s) {
        double q1 = Math.cos(0.5 * Math.PI * s);
        double q2 = Math.sin(0.5 * Math.PI * s);
        return new double[]{q1, q2};
    }

    // dq_p/ds
    static double[] dqdsPath(double s) {
        double dq1 = -0.5 * Math.PI * Math.sin(0.5 * Math.PI * s);
        double dq2 =  0.5 * Math.PI * Math.cos(0.5 * Math.PI * s);
        return new double[]{dq1, dq2};
    }

    // d^2 q_p/ds^2
    static double[] d2qds2Path(double s) {
        double c = 0.5 * Math.PI;
        double d2q1 = -(c * c) * Math.cos(c * s);
        double d2q2 = -(c * c) * Math.sin(c * s);
        return new double[]{d2q1, d2q2};
    }

    static double sTime(double t, double T) {
        double tau = t / T;
        return 3.0 * tau * tau - 2.0 * tau * tau * tau;
    }

    static double sdotTime(double t, double T) {
        double tau = t / T;
        return (6.0 * tau - 6.0 * tau * tau) / T;
    }

    static double sddotTime(double t, double T) {
        double tau = t / T;
        return (6.0 - 12.0 * tau) / (T * T);
    }

    public static void main(String[] args) {
        double T = 2.0;
        int nSamples = 100;
        double qdotMax1 = 1.0;
        double qdotMax2 = 1.0;
        boolean violated = false;

        for (int k = 0; k < nSamples; ++k) {
            double t = (T * k) / (double)(nSamples - 1);
            double s = sTime(t, T);
            double sdot = sdotTime(t, T);
            double sddot = sddotTime(t, T);

            double[] q = qPath(s);
            double[] dqds = dqdsPath(s);
            double[] d2qds2 = d2qds2Path(s);

            double qdot1 = dqds[0] * sdot;
            double qdot2 = dqds[1] * sdot;

            double qddot1 = d2qds2[0] * sdot * sdot + dqds[0] * sddot;
            double qddot2 = d2qds2[1] * sdot * sdot + dqds[1] * sddot;

            if (Math.abs(qdot1) > qdotMax1 || Math.abs(qdot2) > qdotMax2) {
                violated = true;
            }
        }

        if (violated) {
            System.out.println("Joint velocity limits exceeded.");
        } else {
            System.out.println("Joint velocity limits satisfied.");
        }
    }
}
      
