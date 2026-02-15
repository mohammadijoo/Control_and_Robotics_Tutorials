
public interface RobotRegressor {
    // returns Y(q, dq, qr, dqr, ddqr) as n x p matrix
    double[][] regressor(double[] q, double[] dq,
                         double[] qr, double[] dqr, double[] ddqr);
    int numParams();
}

public class AdaptiveCTControllerJava {
    private final int n;
    private final int p;
    private final double[][] Kd;
    private final double[][] Lambda;
    private final double[][] Gamma_d;
    private final RobotRegressor model;
    private final double[] thetaHat;

    public AdaptiveCTControllerJava(int nDof,
                                    double[][] Kd,
                                    double[][] Lambda,
                                    double[][] Gamma_d,
                                    RobotRegressor model) {
        this.n = nDof;
        this.Kd = Kd;
        this.Lambda = Lambda;
        this.Gamma_d = Gamma_d;
        this.model = model;
        this.p = model.numParams();
        this.thetaHat = new double[p];
    }

    public double[] step(double[] q, double[] dq,
                         double[] qd, double[] dqd, double[] ddqd) {
        double[] e  = sub(q, qd);
        double[] de = sub(dq, dqd);

        double[] s = add(de, matVec(Lambda, e));

        double[] dqr  = sub(dqd, matVec(Lambda, e));
        double[] ddqr = sub(ddqd, matVec(Lambda, de));

        double[][] Y = model.regressor(q, dq, dqr, dqr, ddqr);

        double[] Ytheta = matVec(Y, thetaHat);
        double[] Kds    = matVec(Kd, s);

        double[] tau = sub(Ytheta, Kds);

        double[] grad = matTVec(Y, s);
        double[] GammaGrad = matVec(Gamma_d, grad);
        for (int i = 0; i != p; ++i) {
            thetaHat[i] -= GammaGrad[i];
        }
        return tau;
    }

    // helper linear algebra (sub, add, matVec, matTVec) to be implemented
}
