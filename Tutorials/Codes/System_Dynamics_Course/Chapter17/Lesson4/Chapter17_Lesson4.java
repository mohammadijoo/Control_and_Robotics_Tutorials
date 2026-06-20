// Chapter17_Lesson4.java
// Noise Modeling in Sensors and Actuators for Dynamic Systems
// Java simulation of a noisy mass-spring-damper sensor/actuator chain.

import java.util.Random;

public class Chapter17_Lesson4 {

    static class Gaussian {
        private final Random rng;
        Gaussian(long seed) { rng = new Random(seed); }
        double next() {
            // Box-Muller
            double u1 = Math.max(1e-12, rng.nextDouble());
            double u2 = rng.nextDouble();
            return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        }
    }

    public static void main(String[] args) {
        final double m = 1.2, c = 0.35, k = 4.0;
        final double Ts = 0.01;
        final int N = 6000;

        final double sigmaW = 0.20;
        final double sigmaV = 0.01;
        final double sigmaBiasRW = 0.002;
        final double deltaQ = 0.001;
        final double rhoA = 0.97;
        final double sigmaASS = 0.08;

        Gaussian g = new Gaussian(17L);

        double x1 = 0.0, x2 = 0.0;
        double bias = 0.0, aNoise = 0.0;

        double[] y = new double[N];
        double[] yTrue = new double[N];
        double[] syy = new double[N];

        // Discrete model (Euler)
        double A11 = 1.0;
        double A12 = Ts;
        double A21 = -Ts * k / m;
        double A22 = 1.0 - Ts * c / m;
        double B1 = 0.0;
        double B2 = Ts / m;
        double G1 = 0.0;
        double G2 = Ts / m;

        // Covariance recursion
        double p11 = 0.0, p12 = 0.0, p22 = 0.0;
        double Qw = sigmaW * sigmaW;
        double Qa = sigmaASS * sigmaASS;
        double Rv = sigmaV * sigmaV;
        double Rq = deltaQ * deltaQ / 12.0;

        for (int kIdx = 0; kIdx < N - 1; kIdx++) {
            double t = kIdx * Ts;
            double uCmd = 0.8 * Math.sin(2.0 * Math.PI * 0.7 * t);

            aNoise = rhoA * aNoise + Math.sqrt(1.0 - rhoA * rhoA) * sigmaASS * g.next();
            double w = sigmaW * g.next();
            bias += sigmaBiasRW * Math.sqrt(Ts) * g.next();
            double v = sigmaV * g.next();

            double uActual = uCmd + aNoise;
            double x1n = A11 * x1 + A12 * x2 + B1 * uActual + G1 * w;
            double x2n = A21 * x1 + A22 * x2 + B2 * uActual + G2 * w;
            x1 = x1n;
            x2 = x2n;

            double yAnalog = x1 + bias + v;
            double q = deltaQ * Math.rint(yAnalog / deltaQ) - yAnalog;
            y[kIdx + 1] = yAnalog + q;
            yTrue[kIdx + 1] = x1;

            // P <- A P A^T + GQG^T + BQaB^T
            double ap11 = A11 * p11 + A12 * p12;
            double ap12 = A11 * p12 + A12 * p22;
            double ap21 = A21 * p11 + A22 * p12;
            double ap22 = A21 * p12 + A22 * p22;

            double p11n = ap11 * A11 + ap12 * A12 + G1 * Qw * G1 + B1 * Qa * B1;
            double p12n = ap11 * A21 + ap12 * A22 + G1 * Qw * G2 + B1 * Qa * B2;
            double p22n = ap21 * A21 + ap22 * A22 + G2 * Qw * G2 + B2 * Qa * B2;

            p11 = p11n;
            p12 = p12n;
            p22 = p22n;

            syy[kIdx + 1] = p11 + Rv + Rq;
        }

        int burn = 1000;
        int M = N - burn;
        double meanY = 0.0, meanYTrue = 0.0, meanSyy = 0.0;
        for (int i = burn; i < N; i++) {
            meanY += y[i];
            meanYTrue += yTrue[i];
            meanSyy += syy[i];
        }
        meanY /= M;
        meanYTrue /= M;
        meanSyy /= M;

        double varY = 0.0, varYTrue = 0.0;
        for (int i = burn; i < N; i++) {
            varY += (y[i] - meanY) * (y[i] - meanY);
            varYTrue += (yTrue[i] - meanYTrue) * (yTrue[i] - meanYTrue);
        }
        varY /= (M - 1);
        varYTrue /= (M - 1);

        System.out.println("=== Noise Modeling Demo (Java) ===");
        System.out.printf("Empirical var(y_true): %.6e%n", varYTrue);
        System.out.printf("Empirical var(y)    : %.6e%n", varY);
        System.out.printf("Predicted mean Syy  : %.6e (bias RW excluded)%n", meanSyy);
        System.out.printf("Quantization theory : %.6e%n", (deltaQ * deltaQ / 12.0));
    }
}
