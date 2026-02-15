import java.util.List;

class TactileContact {
    double[][] positions; // K x 3
    double[] pressures;   // K
    double[] areas;       // K
    double[] cDes;        // length 3
    double[] nDes;        // length 3
    double mu;
}

interface JacobianProvider {
    // Return 3 x n Jacobian matrix in row-major order
    double[][] Jc(double[] q, int contactIndex);
    double[][] Jn(double[] q, int contactIndex);
}

public class TactileGraspRefiner {
    private final double wSlip;
    private final double wPos;
    private final double wAlign;
    private final double stepSize;

    public TactileGraspRefiner(double wSlip,
                               double wPos,
                               double wAlign,
                               double stepSize) {
        this.wSlip = wSlip;
        this.wPos = wPos;
        this.wAlign = wAlign;
        this.stepSize = stepSize;
    }

    private static double[] centerOfPressure(TactileContact c) {
        int K = c.pressures.length;
        double[] result = new double[4]; // [F_n, cx, cy, cz]
        double F = 0.0;
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (int k = 0; k < K; ++k) {
            double w = c.pressures[k] * c.areas[k];
            F += w;
            cx += w * c.positions[k][0];
            cy += w * c.positions[k][1];
            cz += w * c.positions[k][2];
        }
        F += 1e-9;
        result[0] = F;
        result[1] = cx / F;
        result[2] = cy / F;
        result[3] = cz / F;
        return result;
    }

    private static double slipIndicator(double F_n,
                                        double fTNorm,
                                        double mu) {
        return fTNorm / (mu * F_n + 1e-9) - 1.0;
    }

    public double[] refineOnce(double[] q,
                               List contacts,
                               JacobianProvider jac,
                               double[] qMin,
                               double[] qMax) {
        int n = q.length;
        double[] grad = new double[n];

        for (int j = 0; j < contacts.size(); ++j) {
            TactileContact c = (TactileContact) contacts.get(j);
            double[] cop = centerOfPressure(c);
            double F_n = cop[0];
            double[] cEst = new double[]{cop[1], cop[2], cop[3]};

            double posNorm = 0.0;
            double[] posErr = new double[3];
            for (int i = 0; i < 3; ++i) {
                posErr[i] = cEst[i] - c.cDes[i];
                posNorm += posErr[i] * posErr[i];
            }
            posNorm = Math.sqrt(posNorm);
            double h = slipIndicator(F_n, posNorm, c.mu);
            double slipPenalty = Math.max(0.0, h);

            double[][] Jc = jac.Jc(q, j);
            double[][] Jn = jac.Jn(q, j);

            double[] gradPos = new double[n];
            for (int col = 0; col < n; ++col) {
                double acc = 0.0;
                for (int row = 0; row < 3; ++row) {
                    acc += Jc[row][col] * posErr[row];
                }
                gradPos[col] = 2.0 * acc;
            }

            double[] nEst = new double[]{0.0, 0.0, 0.0};
            double[] nErr = new double[3];
            for (int i = 0; i < 3; ++i) {
                nErr[i] = nEst[i] - c.nDes[i];
            }
            double[] gradAlign = new double[n];
            for (int col = 0; col < n; ++col) {
                double acc = 0.0;
                for (int row = 0; row < 3; ++row) {
                    acc += Jn[row][col] * nErr[row];
                }
                gradAlign[col] = 2.0 * acc;
            }

            double[] gradSlip = new double[n];
            if (slipPenalty > 0.0 && posNorm > 1e-6) {
                double[] dir = new double[3];
                for (int i = 0; i < 3; ++i) {
                    dir[i] = posErr[i] / posNorm;
                }
                for (int col = 0; col < n; ++col) {
                    double acc = 0.0;
                    for (int row = 0; row < 3; ++row) {
                        acc += Jc[row][col] * dir[row];
                    }
                    gradSlip[col] = 2.0 * slipPenalty *
                                    acc / (c.mu * F_n + 1e-9);
                }
            }

            for (int i = 0; i < n; ++i) {
                grad[i] += wPos * gradPos[i]
                         + wAlign * gradAlign[i]
                         + wSlip * gradSlip[i];
            }
        }

        double[] qNew = new double[n];
        for (int i = 0; i < n; ++i) {
            double dq = -stepSize * grad[i];
            double qi = q[i] + dq;
            if (qMin != null) {
                qi = Math.max(qMin[i], qi);
            }
            if (qMax != null) {
                qi = Math.min(qMax[i], qi);
            }
            qNew[i] = qi;
        }
        return qNew;
    }
}
      
