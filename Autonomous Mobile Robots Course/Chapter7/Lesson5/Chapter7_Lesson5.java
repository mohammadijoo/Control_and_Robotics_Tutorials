// Chapter7_Lesson5.java
// EKF/UKF localization lab: Wheel + IMU + GPS (2D ground robot)
// Dependencies: EJML (Efficient Java Matrix Library)
// Build (example, adjust classpath):
//   javac -cp ejml-all-0.43.jar Chapter7_Lesson5.java
//   java  -cp .:ejml-all-0.43.jar Chapter7_Lesson5
//
// Note: This is a teaching reference implementation (not optimized).

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.SpecializedOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecomposition_F64;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;

import java.util.Random;
import java.util.ArrayList;

public class Chapter7_Lesson5 {

    static double wrapAngle(double a) {
        a = (a + Math.PI) % (2.0 * Math.PI);
        if (a < 0) a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    // ------------------------ Simulation ------------------------
    static class SimData {
        double[] t;
        DMatrixRMaj[] Xtrue; // each is 6x1
        double[] imuW;
        double[] imuA;
        int[] wheelIdx;
        double[] wheelV;
        double[] wheelW;
        int[] gpsIdx;
        double[][] gpsXY; // [i][2]
    }

    static SimData simulate(double T, double dtImu, double dtWheel, double dtGps) {
        int n = (int)Math.floor(T / dtImu) + 1;
        SimData d = new SimData();
        d.t = new double[n];
        d.Xtrue = new DMatrixRMaj[n];
        d.imuW = new double[n];
        d.imuA = new double[n];

        Random rng = new Random(7);

        double sigmaG = 0.02, sigmaA = 0.20;
        double sigmaBgRw = 5e-4, sigmaBaRw = 2e-3;
        double sigmaVw = 0.10, sigmaWw = 0.02;
        double sigmaGps = 1.5;

        for (int k = 0; k < n; ++k) d.t[k] = k * dtImu;

        d.Xtrue[0] = new DMatrixRMaj(6,1);
        d.Xtrue[0].set(0,0,0.0);
        d.Xtrue[0].set(1,0,0.0);
        d.Xtrue[0].set(2,0,0.2);
        d.Xtrue[0].set(3,0,1.0);
        d.Xtrue[0].set(4,0,0.02);
        d.Xtrue[0].set(5,0,-0.05);

        for (int k = 0; k < n - 1; ++k) {
            double tt = d.t[k];
            double omegaCmd = 0.25*Math.sin(0.2*tt) + 0.05*Math.sin(1.1*tt);
            double aCmd = 0.4*Math.sin(0.1*tt);

            double x = d.Xtrue[k].get(0,0);
            double y = d.Xtrue[k].get(1,0);
            double th = d.Xtrue[k].get(2,0);
            double v = d.Xtrue[k].get(3,0);
            double bg = d.Xtrue[k].get(4,0);
            double ba = d.Xtrue[k].get(5,0);

            double bgNext = bg + sigmaBgRw*Math.sqrt(dtImu)*rng.nextGaussian();
            double baNext = ba + sigmaBaRw*Math.sqrt(dtImu)*rng.nextGaussian();

            double thNext = wrapAngle(th + omegaCmd*dtImu);
            double vNext  = v + aCmd*dtImu;
            double xNext  = x + v*dtImu*Math.cos(th);
            double yNext  = y + v*dtImu*Math.sin(th);

            d.Xtrue[k+1] = new DMatrixRMaj(6,1);
            d.Xtrue[k+1].set(0,0,xNext);
            d.Xtrue[k+1].set(1,0,yNext);
            d.Xtrue[k+1].set(2,0,thNext);
            d.Xtrue[k+1].set(3,0,vNext);
            d.Xtrue[k+1].set(4,0,bgNext);
            d.Xtrue[k+1].set(5,0,baNext);

            d.imuW[k] = omegaCmd + bg + sigmaG*rng.nextGaussian();
            d.imuA[k] = aCmd + ba + sigmaA*rng.nextGaussian();
        }
        d.imuW[n-1] = d.imuW[n-2];
        d.imuA[n-1] = d.imuA[n-2];

        int stepWheel = Math.max(1, (int)Math.round(dtWheel/dtImu));
        int stepGps = Math.max(1, (int)Math.round(dtGps/dtImu));

        ArrayList<Integer> widx = new ArrayList<>();
        ArrayList<Double> wv = new ArrayList<>();
        ArrayList<Double> ww = new ArrayList<>();
        for (int k = 0; k < n; k += stepWheel) {
            widx.add(k);
            wv.add(d.Xtrue[k].get(3,0) + sigmaVw*rng.nextGaussian());
            // wheel yaw-rate ~ (imuW - bg) with extra noise
            ww.add((d.imuW[k] - d.Xtrue[k].get(4,0)) + sigmaWw*rng.nextGaussian());
        }
        d.wheelIdx = widx.stream().mapToInt(i -> i).toArray();
        d.wheelV = new double[d.wheelIdx.length];
        d.wheelW = new double[d.wheelIdx.length];
        for (int i = 0; i < d.wheelIdx.length; ++i) {
            d.wheelV[i] = wv.get(i);
            d.wheelW[i] = ww.get(i);
        }

        ArrayList<Integer> gidx = new ArrayList<>();
        ArrayList<double[]> gxy = new ArrayList<>();
        for (int k = 0; k < n; k += stepGps) {
            gidx.add(k);
            double[] z = new double[2];
            z[0] = d.Xtrue[k].get(0,0) + sigmaGps*rng.nextGaussian();
            z[1] = d.Xtrue[k].get(1,0) + sigmaGps*rng.nextGaussian();
            gxy.add(z);
        }
        d.gpsIdx = gidx.stream().mapToInt(i -> i).toArray();
        d.gpsXY = gxy.toArray(new double[0][0]);

        return d;
    }

    // ------------------------ EKF ------------------------
    static class EKF {
        DMatrixRMaj x; // 6x1
        DMatrixRMaj P; // 6x6
        DMatrixRMaj Qc; // 4x4 (gyro, accel, bg_rw, ba_rw)
        DMatrixRMaj Rw; // 2x2
        DMatrixRMaj Rg; // 2x2

        EKF(DMatrixRMaj x0, DMatrixRMaj P0, DMatrixRMaj Qc, DMatrixRMaj Rw, DMatrixRMaj Rg) {
            this.x = x0.copy();
            this.P = P0.copy();
            this.Qc = Qc.copy();
            this.Rw = Rw.copy();
            this.Rg = Rg.copy();
        }

        void predict(double omegaM, double aM, double dt) {
            double X = x.get(0,0), Y = x.get(1,0), th = x.get(2,0), v = x.get(3,0), bg = x.get(4,0), ba = x.get(5,0);
            double omega = omegaM - bg;
            double acc = aM - ba;

            DMatrixRMaj xp = new DMatrixRMaj(6,1);
            xp.set(0,0, X + v*dt*Math.cos(th));
            xp.set(1,0, Y + v*dt*Math.sin(th));
            xp.set(2,0, wrapAngle(th + omega*dt));
            xp.set(3,0, v + acc*dt);
            xp.set(4,0, bg);
            xp.set(5,0, ba);
            x.setTo(xp);

            DMatrixRMaj F = CommonOps_DDRM.identity(6);
            F.set(0,2, -v*dt*Math.sin(th));
            F.set(0,3,  dt*Math.cos(th));
            F.set(1,2,  v*dt*Math.cos(th));
            F.set(1,3,  dt*Math.sin(th));
            F.set(2,4, -dt);
            F.set(3,5, -dt);

            DMatrixRMaj G = new DMatrixRMaj(6,4);
            G.set(2,0, dt);
            G.set(3,1, dt);
            G.set(4,2, Math.sqrt(dt));
            G.set(5,3, Math.sqrt(dt));

            DMatrixRMaj Qd = new DMatrixRMaj(6,6);
            DMatrixRMaj tmp = new DMatrixRMaj(6,4);
            CommonOps_DDRM.mult(G, Qc, tmp);
            CommonOps_DDRM.multTransB(tmp, G, Qd);

            DMatrixRMaj Pp = new DMatrixRMaj(6,6);
            DMatrixRMaj tmp2 = new DMatrixRMaj(6,6);
            CommonOps_DDRM.mult(F, P, tmp2);
            CommonOps_DDRM.multTransB(tmp2, F, Pp);
            CommonOps_DDRM.addEquals(Pp, Qd);
            P.setTo(Pp);
        }

        void updateWheel(double vW, double omegaW, double omegaM) {
            DMatrixRMaj z = new DMatrixRMaj(2,1);
            z.set(0,0,vW);
            z.set(1,0,omegaW);

            DMatrixRMaj h = new DMatrixRMaj(2,1);
            h.set(0,0, x.get(3,0));
            h.set(1,0, omegaM - x.get(4,0));

            DMatrixRMaj H = new DMatrixRMaj(2,6);
            H.set(0,3,1.0);
            H.set(1,4,-1.0);

            update(z, h, H, Rw);
        }

        void updateGps(double[] xy) {
            DMatrixRMaj z = new DMatrixRMaj(2,1);
            z.set(0,0,xy[0]);
            z.set(1,0,xy[1]);

            DMatrixRMaj h = new DMatrixRMaj(2,1);
            h.set(0,0,x.get(0,0));
            h.set(1,0,x.get(1,0));

            DMatrixRMaj H = new DMatrixRMaj(2,6);
            H.set(0,0,1.0);
            H.set(1,1,1.0);

            update(z, h, H, Rg);
        }

        void update(DMatrixRMaj z, DMatrixRMaj h, DMatrixRMaj H, DMatrixRMaj R) {
            DMatrixRMaj y = new DMatrixRMaj(z);
            CommonOps_DDRM.subtractEquals(y, h);

            DMatrixRMaj S = new DMatrixRMaj(2,2);
            DMatrixRMaj tmp = new DMatrixRMaj(2,6);
            CommonOps_DDRM.mult(H, P, tmp);
            CommonOps_DDRM.multTransB(tmp, H, S);
            CommonOps_DDRM.addEquals(S, R);

            DMatrixRMaj Sinv = new DMatrixRMaj(2,2);
            CommonOps_DDRM.invert(S, Sinv);

            DMatrixRMaj K = new DMatrixRMaj(6,2);
            DMatrixRMaj tmp2 = new DMatrixRMaj(6,2);
            CommonOps_DDRM.multTransA(P, H, tmp2);
            CommonOps_DDRM.mult(tmp2, Sinv, K);

            DMatrixRMaj dx = new DMatrixRMaj(6,1);
            CommonOps_DDRM.mult(K, y, dx);
            CommonOps_DDRM.addEquals(x, dx);
            x.set(2,0, wrapAngle(x.get(2,0)));

            DMatrixRMaj I = CommonOps_DDRM.identity(6);
            DMatrixRMaj KH = new DMatrixRMaj(6,6);
            CommonOps_DDRM.mult(K, H, KH);

            DMatrixRMaj I_KH = new DMatrixRMaj(6,6);
            CommonOps_DDRM.subtract(I, KH, I_KH);

            DMatrixRMaj Pnew = new DMatrixRMaj(6,6);
            DMatrixRMaj tmp3 = new DMatrixRMaj(6,6);
            CommonOps_DDRM.mult(I_KH, P, tmp3);
            CommonOps_DDRM.multTransB(tmp3, I_KH, Pnew);

            DMatrixRMaj KRKt = new DMatrixRMaj(6,6);
            DMatrixRMaj tmp4 = new DMatrixRMaj(6,2);
            CommonOps_DDRM.mult(K, R, tmp4);
            CommonOps_DDRM.multTransB(tmp4, K, KRKt);
            CommonOps_DDRM.addEquals(Pnew, KRKt);

            P.setTo(Pnew);
        }
    }

    // ------------------------ UKF (compact) ------------------------
    static class UKF {
        DMatrixRMaj x; // 6x1
        DMatrixRMaj P; // 6x6
        DMatrixRMaj Qd; // 6x6
        DMatrixRMaj Rw; // 2x2
        DMatrixRMaj Rg; // 2x2

        int n = 6;
        double alpha = 1e-3, beta = 2.0, kappa = 0.0;
        double lambda;
        double[] Wm, Wc;

        UKF(DMatrixRMaj x0, DMatrixRMaj P0, DMatrixRMaj Qd, DMatrixRMaj Rw, DMatrixRMaj Rg) {
            this.x = x0.copy();
            this.P = P0.copy();
            this.Qd = Qd.copy();
            this.Rw = Rw.copy();
            this.Rg = Rg.copy();

            lambda = alpha*alpha*(n + kappa) - n;
            Wm = new double[2*n + 1];
            Wc = new double[2*n + 1];
            double c = 1.0 / (2.0 * (n + lambda));
            for (int i = 0; i < 2*n + 1; ++i) { Wm[i] = c; Wc[i] = c; }
            Wm[0] = lambda / (n + lambda);
            Wc[0] = Wm[0] + (1.0 - alpha*alpha + beta);
        }

        DMatrixRMaj sigmaPoints(DMatrixRMaj x0, DMatrixRMaj P0) {
            DMatrixRMaj A = new DMatrixRMaj(n,n);
            CommonOps_DDRM.scale(n + lambda, P0, A);
            CholeskyDecomposition_F64<DMatrixRMaj> chol = DecompositionFactory_DDRM.chol(n,true);
            chol.decompose(A);
            DMatrixRMaj S = chol.getT(null); // upper; we will use columns carefully

            DMatrixRMaj X = new DMatrixRMaj(2*n + 1, n);
            for (int j = 0; j < n; ++j) X.set(0,j, x0.get(j,0));
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    double sij = S.get(i,j); // upper triangular
                    X.set(1+i, j, x0.get(j,0) + sij);
                    X.set(1+i+n, j, x0.get(j,0) - sij);
                }
            }
            // wrap theta column
            for (int i = 0; i < X.numRows; ++i) X.set(i,2, wrapAngle(X.get(i,2)));
            return X;
        }

        DMatrixRMaj f(DMatrixRMaj s, double omegaM, double aM, double dt) {
            double X = s.get(0,0), Y = s.get(1,0), th = s.get(2,0), v = s.get(3,0), bg = s.get(4,0), ba = s.get(5,0);
            double omega = omegaM - bg;
            double acc = aM - ba;

            DMatrixRMaj xp = new DMatrixRMaj(6,1);
            xp.set(0,0, X + v*dt*Math.cos(th));
            xp.set(1,0, Y + v*dt*Math.sin(th));
            xp.set(2,0, wrapAngle(th + omega*dt));
            xp.set(3,0, v + acc*dt);
            xp.set(4,0, bg);
            xp.set(5,0, ba);
            return xp;
        }

        double thetaMean(DMatrixRMaj Xp) {
            double s = 0.0, c = 0.0;
            for (int i = 0; i < Xp.numRows; ++i) {
                double th = Xp.get(i,2);
                s += Wm[i] * Math.sin(th);
                c += Wm[i] * Math.cos(th);
            }
            return Math.atan2(s, c);
        }

        void predict(double omegaM, double aM, double dt) {
            DMatrixRMaj X = sigmaPoints(x, P);
            DMatrixRMaj Xp = new DMatrixRMaj(X.numRows, n);
            for (int i = 0; i < X.numRows; ++i) {
                DMatrixRMaj si = new DMatrixRMaj(n,1);
                for (int j = 0; j < n; ++j) si.set(j,0, X.get(i,j));
                DMatrixRMaj yi = f(si, omegaM, aM, dt);
                for (int j = 0; j < n; ++j) Xp.set(i,j, yi.get(j,0));
            }

            DMatrixRMaj xMean = new DMatrixRMaj(n,1);
            for (int i = 0; i < Xp.numRows; ++i) {
                for (int j = 0; j < n; ++j) xMean.add(j,0, Wm[i]*Xp.get(i,j));
            }
            xMean.set(2,0, thetaMean(Xp));

            DMatrixRMaj Pp = new DMatrixRMaj(n,n);
            for (int i = 0; i < Xp.numRows; ++i) {
                double[] dx = new double[n];
                for (int j = 0; j < n; ++j) dx[j] = Xp.get(i,j) - xMean.get(j,0);
                dx[2] = wrapAngle(dx[2]);
                for (int r = 0; r < n; ++r) {
                    for (int c = 0; c < n; ++c) {
                        Pp.add(r,c, Wc[i]*dx[r]*dx[c]);
                    }
                }
            }
            CommonOps_DDRM.addEquals(Pp, Qd);

            x.setTo(xMean);
            P.setTo(Pp);
        }

        void updateWheel(double vW, double omegaW, double omegaM) {
            double[] z = new double[]{vW, omegaW};
            update(z, omegaM, true);
        }

        void updateGps(double[] xy) {
            double[] z = new double[]{xy[0], xy[1]};
            update(z, 0.0, false);
        }

        void update(double[] z, double omegaM, boolean wheel) {
            DMatrixRMaj X = sigmaPoints(x, P);
            int m = 2;
            DMatrixRMaj Z = new DMatrixRMaj(X.numRows, m);

            for (int i = 0; i < X.numRows; ++i) {
                if (wheel) {
                    Z.set(i,0, X.get(i,3));
                    Z.set(i,1, omegaM - X.get(i,4));
                } else {
                    Z.set(i,0, X.get(i,0));
                    Z.set(i,1, X.get(i,1));
                }
            }

            double[] zMean = new double[m];
            for (int i = 0; i < Z.numRows; ++i) {
                zMean[0] += Wm[i]*Z.get(i,0);
                zMean[1] += Wm[i]*Z.get(i,1);
            }

            DMatrixRMaj S = new DMatrixRMaj(m,m);
            DMatrixRMaj Pxz = new DMatrixRMaj(n,m);

            for (int i = 0; i < Z.numRows; ++i) {
                double[] dz = new double[]{Z.get(i,0)-zMean[0], Z.get(i,1)-zMean[1]};
                double[] dx = new double[n];
                for (int j = 0; j < n; ++j) dx[j] = X.get(i,j) - x.get(j,0);
                dx[2] = wrapAngle(dx[2]);

                for (int r = 0; r < m; ++r) {
                    for (int c = 0; c < m; ++c) S.add(r,c, Wc[i]*dz[r]*dz[c]);
                }
                for (int r = 0; r < n; ++r) {
                    for (int c = 0; c < m; ++c) Pxz.add(r,c, Wc[i]*dx[r]*dz[c]);
                }
            }

            if (wheel) CommonOps_DDRM.addEquals(S, Rw);
            else CommonOps_DDRM.addEquals(S, Rg);

            DMatrixRMaj Sinv = new DMatrixRMaj(m,m);
            CommonOps_DDRM.invert(S, Sinv);

            DMatrixRMaj K = new DMatrixRMaj(n,m);
            CommonOps_DDRM.mult(Pxz, Sinv, K);

            // x = x + K*(z - zMean)
            DMatrixRMaj innov = new DMatrixRMaj(m,1);
            innov.set(0,0, z[0]-zMean[0]);
            innov.set(1,0, z[1]-zMean[1]);

            DMatrixRMaj dx = new DMatrixRMaj(n,1);
            CommonOps_DDRM.mult(K, innov, dx);
            CommonOps_DDRM.addEquals(x, dx);
            x.set(2,0, wrapAngle(x.get(2,0)));

            // P = P - K*S*K'
            DMatrixRMaj KS = new DMatrixRMaj(n,m);
            CommonOps_DDRM.mult(K, S, KS);
            DMatrixRMaj KSKt = new DMatrixRMaj(n,n);
            CommonOps_DDRM.multTransB(KS, K, KSKt);
            CommonOps_DDRM.subtractEquals(P, KSKt);
        }
    }

    public static void main(String[] args) {
        double T = 80.0;
        double dtImu = 0.01, dtWheel = 0.05, dtGps = 1.0;

        SimData d = simulate(T, dtImu, dtWheel, dtGps);

        DMatrixRMaj x0 = new DMatrixRMaj(6,1);
        x0.set(0,0,0.5); x0.set(1,0,-1.0); x0.set(2,0,0.0); x0.set(3,0,0.5); x0.set(4,0,0.0); x0.set(5,0,0.0);

        DMatrixRMaj P0 = new DMatrixRMaj(6,6);
        P0.set(0,0,4.0); P0.set(1,1,4.0); P0.set(2,2, Math.pow(20.0*Math.PI/180.0,2));
        P0.set(3,3,1.0); P0.set(4,4, Math.pow(0.05,2)); P0.set(5,5, Math.pow(0.2,2));

        double sigmaG = 0.02, sigmaA = 0.20;
        double sigmaBgRw = 5e-4, sigmaBaRw = 2e-3;

        DMatrixRMaj Qc = new DMatrixRMaj(4,4);
        Qc.set(0,0, sigmaG*sigmaG);
        Qc.set(1,1, sigmaA*sigmaA);
        Qc.set(2,2, sigmaBgRw*sigmaBgRw);
        Qc.set(3,3, sigmaBaRw*sigmaBaRw);

        double sigmaVw = 0.10, sigmaWw = 0.02, sigmaGps = 1.5;
        DMatrixRMaj Rw = new DMatrixRMaj(2,2);
        Rw.set(0,0, sigmaVw*sigmaVw);
        Rw.set(1,1, sigmaWw*sigmaWw);
        DMatrixRMaj Rg = new DMatrixRMaj(2,2);
        Rg.set(0,0, sigmaGps*sigmaGps);
        Rg.set(1,1, sigmaGps*sigmaGps);

        EKF ekf = new EKF(x0, P0, Qc, Rw, Rg);

        DMatrixRMaj Qd = new DMatrixRMaj(6,6);
        Qd.set(2,2, Math.pow(sigmaG*dtImu,2));
        Qd.set(3,3, Math.pow(sigmaA*dtImu,2));
        Qd.set(4,4, Math.pow(sigmaBgRw*Math.sqrt(dtImu),2));
        Qd.set(5,5, Math.pow(sigmaBaRw*Math.sqrt(dtImu),2));

        UKF ukf = new UKF(x0, P0, Qd, Rw, Rg);

        int n = d.t.length;

        int[] wheelMap = new int[n];
        int[] gpsMap = new int[n];
        for (int i = 0; i < n; ++i) { wheelMap[i] = -1; gpsMap[i] = -1; }
        for (int i = 0; i < d.wheelIdx.length; ++i) wheelMap[d.wheelIdx[i]] = i;
        for (int i = 0; i < d.gpsIdx.length; ++i) gpsMap[d.gpsIdx[i]] = i;

        double sePosEkf = 0.0, seThEkf = 0.0;
        double sePosUkf = 0.0, seThUkf = 0.0;

        for (int k = 0; k < n; ++k) {
            double omegaM = d.imuW[k];
            double aM = d.imuA[k];

            ekf.predict(omegaM, aM, dtImu);
            ukf.predict(omegaM, aM, dtImu);

            if (wheelMap[k] >= 0) {
                int j = wheelMap[k];
                ekf.updateWheel(d.wheelV[j], d.wheelW[j], omegaM);
                ukf.updateWheel(d.wheelV[j], d.wheelW[j], omegaM);
            }
            if (gpsMap[k] >= 0) {
                int j = gpsMap[k];
                ekf.updateGps(d.gpsXY[j]);
                ukf.updateGps(d.gpsXY[j]);
            }

            double xt = d.Xtrue[k].get(0,0);
            double yt = d.Xtrue[k].get(1,0);
            double tht = d.Xtrue[k].get(2,0);

            double dxE = ekf.x.get(0,0) - xt;
            double dyE = ekf.x.get(1,0) - yt;
            double dthE = wrapAngle(ekf.x.get(2,0) - tht);

            double dxU = ukf.x.get(0,0) - xt;
            double dyU = ukf.x.get(1,0) - yt;
            double dthU = wrapAngle(ukf.x.get(2,0) - tht);

            sePosEkf += dxE*dxE + dyE*dyE;
            seThEkf  += dthE*dthE;
            sePosUkf += dxU*dxU + dyU*dyU;
            seThUkf  += dthU*dthU;
        }

        System.out.println("Position RMSE (m): EKF=" + Math.sqrt(sePosEkf/n) + " UKF=" + Math.sqrt(sePosUkf/n));
        System.out.println("Heading RMSE (rad): EKF=" + Math.sqrt(seThEkf/n) + " UKF=" + Math.sqrt(seThUkf/n));
    }
}
