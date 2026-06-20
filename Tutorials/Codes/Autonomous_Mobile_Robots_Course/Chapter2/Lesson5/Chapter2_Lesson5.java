// Chapter2_Lesson5.java
/*
Chapter 2 — Wheeled Locomotion Kinematics (Mobile-Specific)
Lesson 5 — Kinematic Calibration of Wheel Parameters

This Java example estimates p = [r_L, r_R, b] for a differential-drive robot
using damped Gauss-Newton on body-frame relative increments.

Recommended linear algebra library: EJML (Efficient Java Matrix Library)
Maven (pom.xml) dependency:
  <dependency>
    <groupId>org.ejml</groupId>
    <artifactId>ejml-simple</artifactId>
    <version>0.43</version>
  </dependency>

Input CSV format (header optional):
  dphi_L, dphi_R, dx_gt, dy_gt, dtheta_gt
*/

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

public class Chapter2_Lesson5 {

    static class DataRow {
        double dphiL, dphiR, dx, dy, dth;
        DataRow(double a, double b, double c, double d, double e) {
            dphiL = a; dphiR = b; dx = c; dy = d; dth = e;
        }
    }

    static double wrapToPi(double a) {
        double pi = Math.PI;
        a = (a + pi) % (2.0 * pi);
        if (a < 0) a += 2.0 * pi;
        return a - pi;
    }

    static double sinc1(double x) {
        if (Math.abs(x) < 1e-6) {
            double x2 = x * x;
            return 1.0 - x2 / 6.0 + (x2 * x2) / 120.0;
        }
        return Math.sin(x) / x;
    }

    static double cosc1(double x) {
        if (Math.abs(x) < 1e-6) {
            double x2 = x * x;
            return x / 2.0 - (x * x2) / 24.0 + (x * x2 * x2) / 720.0;
        }
        return (1.0 - Math.cos(x)) / x;
    }

    static double dsinc1(double x) {
        if (Math.abs(x) < 1e-5) {
            double x2 = x * x;
            return -(x) / 3.0 + (x * x2) / 30.0 - (x * x2 * x2) / 840.0;
        }
        return (x * Math.cos(x) - Math.sin(x)) / (x * x);
    }

    static double dcosc1(double x) {
        if (Math.abs(x) < 1e-5) {
            double x2 = x * x;
            return 0.5 - x2 / 8.0 + (x2 * x2) / 144.0;
        }
        return (x * Math.sin(x) - (1.0 - Math.cos(x))) / (x * x);
    }

    static List<DataRow> loadCSV(String path) throws Exception {
        List<DataRow> data = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;
                if (line.toLowerCase().contains("dphi")) continue;
                String[] toks = line.split(",");
                if (toks.length < 5) continue;
                double dphiL = Double.parseDouble(toks[0].trim());
                double dphiR = Double.parseDouble(toks[1].trim());
                double dx = Double.parseDouble(toks[2].trim());
                double dy = Double.parseDouble(toks[3].trim());
                double dth = Double.parseDouble(toks[4].trim());
                data.add(new DataRow(dphiL, dphiR, dx, dy, dth));
            }
        }
        return data;
    }

    static class ResJac {
        SimpleMatrix r;  // (3N x 1)
        SimpleMatrix J;  // (3N x 3)
        ResJac(SimpleMatrix r, SimpleMatrix J) { this.r = r; this.J = J; }
    }

    static ResJac residualAndJac(List<DataRow> data, double rL, double rR, double b) {
        int N = data.size();
        SimpleMatrix r = new SimpleMatrix(3*N, 1);
        SimpleMatrix J = new SimpleMatrix(3*N, 3);

        for (int k = 0; k < N; k++) {
            DataRow d = data.get(k);
            double sL = rL * d.dphiL;
            double sR = rR * d.dphiR;
            double A = sR + sL;
            double B = sR - sL;

            double ds = 0.5 * A;
            double dth = B / b;

            double f = sinc1(dth);
            double g = cosc1(dth);
            double fp = dsinc1(dth);
            double gp = dcosc1(dth);

            double dx = ds * f;
            double dy = ds * g;

            double ex = dx - d.dx;
            double ey = dy - d.dy;
            double eth = wrapToPi(dth - d.dth);

            r.set(3*k+0, 0, ex);
            r.set(3*k+1, 0, ey);
            r.set(3*k+2, 0, eth);

            double dds_drL = 0.5 * d.dphiL;
            double dds_drR = 0.5 * d.dphiR;

            double ddth_drL = -(d.dphiL) / b;
            double ddth_drR = (d.dphiR) / b;
            double ddth_db  = -(B) / (b * b);

            double ddx_drL = dds_drL * f + ds * fp * ddth_drL;
            double ddx_drR = dds_drR * f + ds * fp * ddth_drR;
            double ddx_db  = 0.0      * f + ds * fp * ddth_db;

            double ddy_drL = dds_drL * g + ds * gp * ddth_drL;
            double ddy_drR = dds_drR * g + ds * gp * ddth_drR;
            double ddy_db  = 0.0      * g + ds * gp * ddth_db;

            J.set(3*k+0, 0, ddx_drL);
            J.set(3*k+0, 1, ddx_drR);
            J.set(3*k+0, 2, ddx_db);

            J.set(3*k+1, 0, ddy_drL);
            J.set(3*k+1, 1, ddy_drR);
            J.set(3*k+1, 2, ddy_db);

            J.set(3*k+2, 0, ddth_drL);
            J.set(3*k+2, 1, ddth_drR);
            J.set(3*k+2, 2, ddth_db);
        }
        return new ResJac(r, J);
    }

    static double[] calibrateGN(List<DataRow> data, double[] p0) {
        double rL = p0[0], rR = p0[1], b = p0[2];
        double lambda = 1e-3;

        for (int it = 0; it < 60; it++) {
            ResJac rj = residualAndJac(data, rL, rR, b);
            SimpleMatrix r = rj.r;
            SimpleMatrix J = rj.J;

            SimpleMatrix H = J.transpose().mult(J);
            SimpleMatrix g = J.transpose().mult(r);

            SimpleMatrix A = H.plus(SimpleMatrix.identity(3).scale(lambda));
            SimpleMatrix dp = A.solve(g).scale(-1.0);

            double stepNorm = dp.normF();
            if (stepNorm < 1e-12) break;

            double rL2 = rL + dp.get(0,0);
            double rR2 = rR + dp.get(1,0);
            double b2  = b  + dp.get(2,0);

            ResJac rj2 = residualAndJac(data, rL2, rR2, b2);
            double sse2 = rj2.r.dot(rj2.r);
            double sse1 = r.dot(r);

            if (sse2 < sse1) {
                rL = rL2; rR = rR2; b = b2;
                lambda *= 0.7;
            } else {
                lambda *= 2.0;
            }
        }
        return new double[]{rL, rR, b};
    }

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("Usage: java Chapter2_Lesson5 path/to/data.csv");
            System.exit(1);
        }
        List<DataRow> data = loadCSV(args[0]);
        double[] p0 = new double[]{0.05, 0.05, 0.30};
        double[] phat = calibrateGN(data, p0);

        System.out.printf("Estimated [rL, rR, b] = %.8f %.8f %.8f%n", phat[0], phat[1], phat[2]);

        ResJac rj = residualAndJac(data, phat[0], phat[1], phat[2]);
        int N = data.size();
        double rms_dx = 0, rms_dy = 0, rms_th = 0;
        for (int k = 0; k < N; k++) {
            rms_dx += Math.pow(rj.r.get(3*k+0,0), 2);
            rms_dy += Math.pow(rj.r.get(3*k+1,0), 2);
            rms_th += Math.pow(rj.r.get(3*k+2,0), 2);
        }
        rms_dx = Math.sqrt(rms_dx / N);
        rms_dy = Math.sqrt(rms_dy / N);
        rms_th = Math.sqrt(rms_th / N);
        System.out.printf("RMS [dx, dy, dtheta] = %.6e %.6e %.6e%n", rms_dx, rms_dy, rms_th);
    }
}
