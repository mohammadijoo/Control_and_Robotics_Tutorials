// Chapter5_Lesson3.java
// Autonomous Mobile Robots (Control Engineering) — Chapter 5, Lesson 3
// Topic: Drift Sources and Bias Accumulation
//
// Compile:
//   javac Chapter5_Lesson3.java
// Run:
//   java Chapter5_Lesson3
//
// This program mirrors the C++ simulation at a basic level.

import java.util.Random;

public class Chapter5_Lesson3 {

    static double wrapPi(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0*pi);
        if (a < 0) a += 2.0*pi;
        return a - pi;
    }

    static class Result {
        double posErrWheel, posErrGyro, headErrWheelDeg, headErrGyroDeg;
    }

    static Result simulate(
        double T, double dt,
        double rTrue, double bTrue, int ticksPerRev,
        double rHat, double bHat,
        double epsRL, double epsRR,
        double encTickNoiseStd,
        double gyroBias, double gyroNoiseStd,
        long seed
    ) {
        int N = (int)Math.floor(T/dt);
        Random rng = new Random(seed);

        double radPerTick = 2.0*Math.PI / (double)ticksPerRev;
        double rLTrue = rTrue*(1.0 + epsRL);
        double rRTrue = rTrue*(1.0 + epsRR);

        double x=0, y=0, th=0;
        double xw=0, yw=0, thw=0;
        double xg=0, yg=0, thg=0;
        double thGyro=0;

        for (int k=1; k<N; k++) {
            double t = (k-1)*dt;
            double v = 0.6;
            double w = 0.10*Math.sin(2.0*Math.PI*t/20.0);

            double wR = (v + 0.5*bTrue*w)/rRTrue;
            double wL = (v - 0.5*bTrue*w)/rLTrue;

            double dphiRTrue = wR*dt;
            double dphiLTrue = wL*dt;

            double ticksR = dphiRTrue / radPerTick;
            double ticksL = dphiLTrue / radPerTick;

            // Gaussian noise using Box-Muller
            double nR = Math.sqrt(-2.0*Math.log(Math.max(1e-12, rng.nextDouble()))) * Math.cos(2.0*Math.PI*rng.nextDouble());
            double nL = Math.sqrt(-2.0*Math.log(Math.max(1e-12, rng.nextDouble()))) * Math.cos(2.0*Math.PI*rng.nextDouble());

            double ticksRMeas = Math.rint(ticksR) + encTickNoiseStd*nR;
            double ticksLMeas = Math.rint(ticksL) + encTickNoiseStd*nL;

            double dphiRMeas = ticksRMeas * radPerTick;
            double dphiLMeas = ticksLMeas * radPerTick;

            // Ground truth update
            double dSRTrue = rRTrue*dphiRTrue;
            double dSLTrue = rLTrue*dphiLTrue;
            double dSTrue  = 0.5*(dSRTrue + dSLTrue);
            double dThTrue = (dSRTrue - dSLTrue)/bTrue;

            th = wrapPi(th + dThTrue);
            x  = x + dSTrue*Math.cos(th - 0.5*dThTrue);
            y  = y + dSTrue*Math.sin(th - 0.5*dThTrue);

            // Wheel-only estimate
            double dSRHat = rHat*dphiRMeas;
            double dSLHat = rHat*dphiLMeas;
            double dSHat  = 0.5*(dSRHat + dSLHat);
            double dThHat = (dSRHat - dSLHat)/bHat;

            thw = wrapPi(thw + dThHat);
            xw  = xw + dSHat*Math.cos(thw - 0.5*dThHat);
            yw  = yw + dSHat*Math.sin(thw - 0.5*dThHat);

            // Gyro heading
            double nG = Math.sqrt(-2.0*Math.log(Math.max(1e-12, rng.nextDouble()))) * Math.cos(2.0*Math.PI*rng.nextDouble());
            double wMeas = w + gyroBias + gyroNoiseStd*nG;
            thGyro = wrapPi(thGyro + wMeas*dt);

            thg = thGyro;
            xg = xg + dSHat*Math.cos(thg);
            yg = yg + dSHat*Math.sin(thg);
        }

        Result out = new Result();
        out.posErrWheel = Math.sqrt((xw-x)*(xw-x) + (yw-y)*(yw-y));
        out.posErrGyro  = Math.sqrt((xg-x)*(xg-x) + (yg-y)*(yg-y));
        out.headErrWheelDeg = (180.0/Math.PI)*wrapPi(thw-th);
        out.headErrGyroDeg  = (180.0/Math.PI)*wrapPi(thg-th);
        return out;
    }

    public static void main(String[] args) {
        Result r = simulate(
            60.0, 0.01,
            0.05, 0.30, 2048,
            0.05*1.005, 0.30*0.995,
            +0.002, -0.002,
            0.2,
            0.005, 0.002,
            7L
        );

        System.out.println("Final position error (wheel-only) [m]: " + r.posErrWheel);
        System.out.println("Final position error (gyro-heading) [m]: " + r.posErrGyro);
        System.out.println("Final heading error (wheel-only) [deg]: " + r.headErrWheelDeg);
        System.out.println("Final heading error (gyro-heading) [deg]: " + r.headErrGyroDeg);
    }
}
