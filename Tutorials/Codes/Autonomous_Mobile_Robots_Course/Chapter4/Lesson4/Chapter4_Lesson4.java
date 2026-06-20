// Chapter4_Lesson4.java
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 4: Mobile Robot Dynamics (Applied)
Lesson 4: Stability and Tip-Over Risk (vehicle view)

Java implementation of quasi-static tip-over and sliding checks.

Robotics ecosystem notes:
- For ROS integration, consider rosjava (ROS 1) or bridges for ROS 2 ecosystems.
- For geometry/vector math: EJML or JOML.

This file is self-contained and focuses on core computations.
*/

import java.util.Arrays;

public class Chapter4_Lesson4 {

    static class VehicleParams {
        public final double track;      // [m]
        public final double wheelbase;  // [m]
        public final double cgHeight;   // [m]
        public final double mu;         // [-]
        public final double g;          // [m/s^2]

        public VehicleParams(double track, double wheelbase, double cgHeight, double mu, double g) {
            this.track = track;
            this.wheelbase = wheelbase;
            this.cgHeight = cgHeight;
            this.mu = mu;
            this.g = g;
        }
    }

    static double tipThresholdLateral(VehicleParams p) {
        return p.g * (p.track / (2.0 * p.cgHeight));
    }

    static double slideThreshold(VehicleParams p) {
        return p.mu * p.g;
    }

    static double ltrLateral(VehicleParams p, double aY) {
        return 2.0 * p.cgHeight * aY / (p.g * p.track);
    }

    static double lateralAccel(double v, double kappa) {
        return v * v * kappa;
    }

    static double[] linspace(double a, double b, int n) {
        double[] x = new double[n];
        for (int i = 0; i < n; i++) {
            x[i] = a + (b - a) * ((double)i / (n - 1));
        }
        return x;
    }

    public static void main(String[] args) {
        VehicleParams p = new VehicleParams(0.55, 0.65, 0.25, 0.70, 9.81);

        System.out.println("=== Vehicle params ===");
        System.out.println("track=" + p.track + " wheelbase=" + p.wheelbase
                + " cgHeight=" + p.cgHeight + " mu=" + p.mu);
        System.out.println("a_tip_y=" + tipThresholdLateral(p) + " [m/s^2]");
        System.out.println("mu*g=" + slideThreshold(p) + " [m/s^2]");

        int N = 401;
        double[] t = linspace(0.0, 10.0, N);
        double[] v = new double[N];
        double[] kappa = new double[N];

        for (int i = 0; i < N; i++) {
            double ti = t[i];

            // Speed profile
            double vi = 0.2 + 1.5 * (1.0 - Math.exp(-ti / 2.2));
            if (ti > 7.0) vi *= (1.0 - 0.35 * (ti - 7.0) / 3.0);
            v[i] = Math.max(0.0, Math.min(1.9, vi));

            // Curvature profile
            double ki = 0.05 + 0.35 * Math.exp(-Math.pow((ti - 5.0) / 1.8, 2.0));
            kappa[i] = ki;
        }

        double minMarginTipY = 1e9;
        double minMarginSlide = 1e9;

        for (int i = 1; i < N; i++) {
            double dt = t[i] - t[i - 1];
            double aX = (v[i] - v[i - 1]) / dt;
            double aY = lateralAccel(v[i], kappa[i]);

            double LTRy = ltrLateral(p, aY);
            double marginTipY = 1.0 - Math.abs(LTRy);

            double aPlanar = Math.sqrt(aX * aX + aY * aY);
            double marginSlide = slideThreshold(p) - aPlanar;

            minMarginTipY = Math.min(minMarginTipY, marginTipY);
            minMarginSlide = Math.min(minMarginSlide, marginSlide);
        }

        System.out.println("Min margin_tip_y = " + minMarginTipY);
        System.out.println("Min margin_slide = " + minMarginSlide);
    }
}
