// Chapter7_Lesson3.java
// Consistency and Linearization Pitfalls — EKF + NEES/NIS (Java version).
//
// Dependencies (Maven examples):
//   <dependency>
//     <groupId>org.ejml</groupId>
//     <artifactId>ejml-simple</artifactId>
//     <version>0.43.1</version>
//   </dependency>
//   <dependency>
//     <groupId>org.apache.commons</groupId>
//     <artifactId>commons-math3</artifactId>
//     <version>3.6.1</version>
//   </dependency>
//
// Run idea:
//   javac -cp "ejml-simple.jar:commons-math3.jar:." Chapter7_Lesson3.java
//   java  -cp "ejml-simple.jar:commons-math3.jar:." Chapter7_Lesson3

import java.util.Random;
import org.ejml.simple.SimpleMatrix;
import org.apache.commons.math3.distribution.ChiSquaredDistribution;

public class Chapter7_Lesson3 {

    static double wrapAngle(double a) {
        a = (a + Math.PI) % (2.0 * Math.PI);
        if (a < 0) a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    static double[] fMotion(double[] x, double[] u, double dt) {
        double px = x[0], py = x[1], th = x[2];
        double v = u[0], om = u[1];
        return new double[] {
            px + v * dt * Math.cos(th),
            py + v * dt * Math.sin(th),
            wrapAngle(th + om * dt)
        };
    }

    static SimpleMatrix jacF(double[] x, double[] u, double dt) {
        double th = x[2];
        double v  = u[0];
        SimpleMatrix F = SimpleMatrix.identity(3);
        F.set(0, 2, -v * dt * Math.sin(th));
        F.set(1, 2,  v * dt * Math.cos(th));
        return F;
    }

    static double[] hMeas(double[] x, double[] lm) {
        double px = x[0], py = x[1], th = x[2];
        double dx = lm[0] - px;
        double dy = lm[1] - py;
        double r  = Math.hypot(dx, dy);
        double b  = wrapAngle(Math.atan2(dy, dx) - th);
        return new double[] {r, b};
    }

    static SimpleMatrix jacH(double[] x, double[] lm) {
        double px = x[0], py = x[1];
        double dx = lm[0] - px;
        double dy = lm[1] - py;
        double r2 = dx*dx + dy*dy;
        double r  = Math.sqrt(Math.max(r2, 1e-12));

        SimpleMatrix H = new SimpleMatrix(2,3);
        H.set(0,0, -dx / r);
        H.set(0,1, -dy / r);
        H.set(1,0,  dy / r2);
        H.set(1,1, -dx / r2);
        H.set(1,2, -1.0);
        return H;
    }

    static double[] sampleGaussian(SimpleMatrix Cov, Random rng) {
        // Cholesky (SimpleMatrix has chol()).
        SimpleMatrix L = Cov.chol().getT(); // lower-triangular
        double[] z = new double[Cov.numRows()];
        for (int i=0; i<z.length; i++) z[i] = rng.nextGaussian();
        SimpleMatrix zs = new SimpleMatrix(z.length, 1, true, z);
        SimpleMatrix s = L.mult(zs);
        double[] out = new double[z.length];
        for (int i=0;i<z.length;i++) out[i] = s.get(i,0);
        return out;
    }

    static class Stats {
        double anees;
        double anis;
        int N;
    }

    static Stats runMC(double dt, int steps, int trials, long seed) {
        Random rng = new Random(seed);

        SimpleMatrix Q = new SimpleMatrix(3,3);
        Q.set(0,0, 0.08*0.08);
        Q.set(1,1, 0.08*0.08);
        Q.set(2,2, Math.pow(3.0*Math.PI/180.0, 2));

        SimpleMatrix R = new SimpleMatrix(2,2);
        R.set(0,0, 0.15*0.15);
        R.set(1,1, Math.pow(2.0*Math.PI/180.0, 2));

        double[][] landmarks = new double[][] {
            {5.0, 0.0},
            {0.0, 5.0},
            {5.0, 5.0}
        };

        double neesSum = 0.0;
        double nisSum  = 0.0;
        int count = 0;

        for (int tr=0; tr<trials; tr++) {
            double[] xTrue = new double[] {0.0, 0.0, 0.0};

            double[] xHat  = new double[] {0.5, -0.4, 10.0*Math.PI/180.0};
            SimpleMatrix P = new SimpleMatrix(3,3);
            P.set(0,0, 0.8*0.8);
            P.set(1,1, 0.8*0.8);
            P.set(2,2, Math.pow(15.0*Math.PI/180.0, 2));

            for (int k=0; k<steps; k++) {
                double v  = 1.0 + 0.2*Math.sin(0.05*k);
                double om = 0.35 + 0.25*Math.sin(0.03*k);
                double[] u = new double[] {v, om};

                // true propagation
                double[] w = sampleGaussian(Q, rng);
                double[] xNext = fMotion(xTrue, u, dt);
                xTrue = new double[] {xNext[0] + w[0], xNext[1] + w[1], wrapAngle(xNext[2] + w[2])};

                double[] lm = landmarks[k % landmarks.length];

                // measurement
                double[] z = hMeas(xTrue, lm);
                double[] vMeas = sampleGaussian(R, rng);
                z = new double[] {z[0] + vMeas[0], wrapAngle(z[1] + vMeas[1])};

                // EKF predict
                SimpleMatrix F = jacF(xHat, u, dt);
                double[] xPredArr = fMotion(xHat, u, dt);
                SimpleMatrix xPred = new SimpleMatrix(3,1,true, xPredArr);
                SimpleMatrix PPred = F.mult(P).mult(F.transpose()).plus(Q);

                // EKF update
                SimpleMatrix H = jacH(xPredArr, lm);
                double[] zPredArr = hMeas(xPredArr, lm);
                SimpleMatrix zPred = new SimpleMatrix(2,1,true, zPredArr);

                SimpleMatrix zMat = new SimpleMatrix(2,1,true, z);
                SimpleMatrix nu = zMat.minus(zPred);
                nu.set(1,0, wrapAngle(nu.get(1,0)));

                SimpleMatrix S = H.mult(PPred).mult(H.transpose()).plus(R);
                SimpleMatrix K = PPred.mult(H.transpose()).mult(S.invert());

                SimpleMatrix xUpd = xPred.plus(K.mult(nu));
                xUpd.set(2,0, wrapAngle(xUpd.get(2,0)));

                SimpleMatrix I = SimpleMatrix.identity(3);
                P = (I.minus(K.mult(H))).mult(PPred).mult((I.minus(K.mult(H))).transpose()).plus(K.mult(R).mult(K.transpose()));

                // store
                xHat = new double[] {xUpd.get(0,0), xUpd.get(1,0), xUpd.get(2,0)};

                // NEES/NIS
                double[] e = new double[] {xTrue[0]-xHat[0], xTrue[1]-xHat[1], wrapAngle(xTrue[2]-xHat[2])};
                SimpleMatrix eMat = new SimpleMatrix(3,1,true,e);

                double nees = eMat.transpose().mult(P.invert()).mult(eMat).get(0,0);
                double nis  = nu.transpose().mult(S.invert()).mult(nu).get(0,0);

                neesSum += nees;
                nisSum  += nis;
                count++;
            }
        }

        Stats st = new Stats();
        st.anees = neesSum / count;
        st.anis  = nisSum  / count;
        st.N = count;
        return st;
    }

    static double[] chi2BoundsAverage(int dof, int N, double alpha) {
        ChiSquaredDistribution dist = new ChiSquaredDistribution(N * dof);
        double lo = dist.inverseCumulativeProbability(alpha/2.0) / N;
        double hi = dist.inverseCumulativeProbability(1.0 - alpha/2.0) / N;
        return new double[] {lo, hi};
    }

    public static void main(String[] args) {
        int steps = 200;
        int trials = 30;
        double alpha = 0.05;

        double[] dts = new double[] {0.05, 0.20};
        for (double dt : dts) {
            System.out.println("\n=== dt=" + dt + " ===");
            Stats st = runMC(dt, steps, trials, 1);

            System.out.printf("EKF ANEES=%.3f (expected ~ 3)%n", st.anees);
            System.out.printf("EKF ANIS =%.3f (expected ~ 2)%n", st.anis);

            double[] bNees = chi2BoundsAverage(3, st.N, alpha);
            double[] bNis  = chi2BoundsAverage(2, st.N, alpha);
            System.out.printf("95%% bounds for ANEES: [%.3f, %.3f]%n", bNees[0], bNees[1]);
            System.out.printf("95%% bounds for ANIS : [%.3f, %.3f]%n", bNis[0], bNis[1]);

            System.out.println("Interpretation: statistics above the upper bound indicate overconfidence (inconsistency).");
        }
    }
}
