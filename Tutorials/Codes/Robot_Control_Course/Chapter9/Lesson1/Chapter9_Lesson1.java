
public class RobotCost {

    public static double trackingCost(double[][] q,
                                      double[][] qd,
                                      double[] Qdiag,
                                      double dt) {
        int Np1 = q.length;
        int n = q[0].length;
        double J = 0.0;
        for (int k = 0; k < Np1; ++k) {
            for (int i = 0; i < n; ++i) {
                double e = q[k][i] - qd[k][i];
                J += Qdiag[i] * e * e;
            }
        }
        return 0.5 * dt * J;
    }

    public static double effortCost(double[][] tau,
                                    double[] Rdiag,
                                    double dt) {
        int N = tau.length;
        int n = tau[0].length;
        double J = 0.0;
        for (int k = 0; k < N; ++k) {
            for (int i = 0; i < n; ++i) {
                double t = tau[k][i];
                J += Rdiag[i] * t * t;
            }
        }
        return 0.5 * dt * J;
    }

    public static double smoothnessCost(double[][] tau,
                                        double[] SuDiag) {
        int N = tau.length;
        if (N < 2) return 0.0;
        int n = tau[0].length;
        double J = 0.0;
        for (int k = 1; k < N; ++k) {
            for (int i = 0; i < n; ++i) {
                double dtau = tau[k][i] - tau[k - 1][i];
                J += SuDiag[i] * dtau * dtau;
            }
        }
        return 0.5 * J;
    }
}
