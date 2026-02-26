public class KalmanFilterCV2D {
    private final double dt;
    private final double[][] F = new double[4][4];
    private final double[][] H = new double[2][4];
    private final double[][] Q = new double[4][4];
    private final double[][] R = new double[2][2];
    private final double[] x = new double[4];
    private final double[][] P = new double[4][4];

    public KalmanFilterCV2D(double dt, double processVar, double measVar) {
        this.dt = dt;

        // Initialize F as identity with dt in velocity terms
        for (int i = 0; i < 4; ++i) {
            F[i][i] = 1.0;
        }
        F[0][2] = dt;
        F[1][3] = dt;

        // Measurement matrix
        H[0][0] = 1.0;
        H[1][1] = 1.0;

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt2 * dt2;
        double q = processVar;

        double[][] Q1d = {
            {0.25 * dt4, 0.5 * dt3},
            {0.5 * dt3,  dt2}
        };

        zeroMatrix(Q);
        addBlock(Q, Q1d, 0, 0, q);
        addBlock(Q, Q1d, 2, 2, q);

        R[0][0] = measVar;
        R[1][1] = measVar;

        identityMatrix(P, 1e3);
    }

    // Helper methods zeroMatrix, identityMatrix, addBlock, matMul, matAdd, matSub,
    // matInv2x2 would be implemented with straightforward loops.

    public void predict() {
        double[] xNew = matMul(F, x);
        double[][] PNew = matAdd(matMul(F, matMul(P, transpose(F))), Q);
        System.arraycopy(xNew, 0, x, 0, 4);
        copyMatrix(PNew, P);
    }

    public void update(double[] z) {
        double[] zPred = matMul(H, x);
        double[] y = matSub(z, zPred);
        double[][] S = matAdd(matMul(H, matMul(P, transpose(H))), R);
        double[][] SInv = matInv2x2(S);
        double[][] K = matMul(matMul(P, transpose(H)), SInv);

        double[] Ky = matMul(K, y);
        for (int i = 0; i < 4; ++i) {
            x[i] += Ky[i];
        }

        double[][] I = identityMatrix(4, 1.0);
        double[][] KH = matMul(K, H);
        double[][] PNew = matMul(matSub(I, KH), P);
        copyMatrix(PNew, P);
    }

    public double[] position() {
        return new double[] {x[0], x[1]};
    }
}
      
