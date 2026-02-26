/*
Chapter5_Lesson5.java

Lab: Characterizing Odometry Error (Differential Drive)

This Java program:
1) Loads a CSV log with: t,nL,nR,x_odom,y_odom,th_odom,x_gt,y_gt,th_gt,(optional)trial_id.
2) Computes endpoint error, RMSE, and drift-per-meter per trial using "logged odom".
3) Calibrates (rL, rR, b) by Gauss-Newton on increment residuals (using ground truth).
4) Reconstructs odometry from encoders using calibrated params and re-evaluates metrics.

Compile:
  javac Chapter5_Lesson5.java

Run:
  java Chapter5_Lesson5 --csv your_log.csv --ticks_per_rev 4096 --r_nom 0.05 --b_nom 0.30 --w_theta 1.0
*/

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

public class Chapter5_Lesson5 {

    static class Params {
        double rL, rR, b;
        Params(double rL, double rR, double b) { this.rL = rL; this.rR = rR; this.b = b; }
    }

    static class Row {
        double t;
        double nL, nR;
        double xO, yO, thO;
        double xG, yG, thG;
        int trialId;
    }

    static class Metrics {
        int trialId;
        double rmsePos, rmseTh, endPos, endTh, sTotal, driftPerM;
    }

    static double wrapAngle(double th) {
        double a = (th + Math.PI) % (2.0 * Math.PI);
        if (a < 0) a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    static ArrayList<Row> loadCSV(String path) throws Exception {
        ArrayList<Row> rows = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String header = br.readLine();
            if (header == null) throw new RuntimeException("Empty CSV");
            String[] cols = header.split(",");
            Map<String,Integer> idx = new HashMap<>();
            for (int i = 0; i < cols.length; i++) idx.put(cols[i].trim(), i);

            String[] req = {"t","nL","nR","x_odom","y_odom","th_odom","x_gt","y_gt","th_gt"};
            for (String r : req) {
                if (!idx.containsKey(r)) throw new RuntimeException("Missing required column: " + r);
            }
            boolean hasTrial = idx.containsKey("trial_id");

            String line;
            while ((line = br.readLine()) != null) {
                if (line.trim().isEmpty()) continue;
                String[] tok = line.split(",");
                Row r = new Row();
                r.t  = Double.parseDouble(tok[idx.get("t")].trim());
                r.nL = Double.parseDouble(tok[idx.get("nL")].trim());
                r.nR = Double.parseDouble(tok[idx.get("nR")].trim());
                r.xO = Double.parseDouble(tok[idx.get("x_odom")].trim());
                r.yO = Double.parseDouble(tok[idx.get("y_odom")].trim());
                r.thO = wrapAngle(Double.parseDouble(tok[idx.get("th_odom")].trim()));
                r.xG = Double.parseDouble(tok[idx.get("x_gt")].trim());
                r.yG = Double.parseDouble(tok[idx.get("y_gt")].trim());
                r.thG = wrapAngle(Double.parseDouble(tok[idx.get("th_gt")].trim()));
                r.trialId = hasTrial ? (int)Math.round(Double.parseDouble(tok[idx.get("trial_id")].trim())) : 0;
                rows.add(r);
            }
        }
        return rows;
    }

    static Metrics computeMetricsLogged(ArrayList<Row> v, int trialId) {
        int N = v.size();
        double sumPos2 = 0.0;
        double sumTh2 = 0.0;

        for (int k = 0; k < N; k++) {
            Row r = v.get(k);
            double ex = r.xO - r.xG;
            double ey = r.yO - r.yG;
            double eth = wrapAngle(r.thO - r.thG);
            double epos = Math.sqrt(ex*ex + ey*ey);
            sumPos2 += epos*epos;
            sumTh2 += eth*eth;
        }

        Metrics m = new Metrics();
        m.trialId = trialId;
        m.rmsePos = Math.sqrt(sumPos2 / Math.max(1, N));
        m.rmseTh  = Math.sqrt(sumTh2 / Math.max(1, N));

        Row rN = v.get(N-1);
        double exN = rN.xO - rN.xG;
        double eyN = rN.yO - rN.yG;
        m.endPos = Math.sqrt(exN*exN + eyN*eyN);
        m.endTh  = Math.abs(wrapAngle(rN.thO - rN.thG));

        double s = 0.0;
        for (int k = 1; k < N; k++) {
            Row a = v.get(k-1);
            Row b = v.get(k);
            double dx = b.xG - a.xG;
            double dy = b.yG - a.yG;
            s += Math.sqrt(dx*dx + dy*dy);
        }
        m.sTotal = s;
        m.driftPerM = (s > 1e-12) ? (m.endPos / s) : Double.NaN;
        return m;
    }

    static double ticksToDphi(double dn, double ticksPerRev) {
        return (2.0 * Math.PI / ticksPerRev) * dn;
    }

    static class IncData {
        double[] dphiL, dphiR, thPrev, dxGt, dyGt, dthGt;
    }

    static IncData buildIncrements(ArrayList<Row> v, double ticksPerRev) {
        int N = v.size();
        IncData d = new IncData();
        d.dphiL = new double[N];
        d.dphiR = new double[N];
        d.thPrev = new double[N];
        d.dxGt = new double[N];
        d.dyGt = new double[N];
        d.dthGt = new double[N];

        d.thPrev[0] = v.get(0).thG;
        for (int k = 1; k < N; k++) {
            Row a = v.get(k-1);
            Row b = v.get(k);
            d.dphiL[k] = ticksToDphi(b.nL - a.nL, ticksPerRev);
            d.dphiR[k] = ticksToDphi(b.nR - a.nR, ticksPerRev);
            d.dxGt[k] = b.xG - a.xG;
            d.dyGt[k] = b.yG - a.yG;
            d.dthGt[k] = wrapAngle(b.thG - a.thG);
            d.thPrev[k] = a.thG;
        }
        return d;
    }

    static class ResidualJac {
        double[] r;   // length 3*(N-1)
        double[] J;   // row-major (3*(N-1) x 3)
    }

    static ResidualJac stackResidualJac(ArrayList<Row> v, double ticksPerRev, Params p, double wTheta) {
        IncData d = buildIncrements(v, ticksPerRev);
        int Nfull = v.size();
        int N = Nfull - 1;

        ResidualJac out = new ResidualJac();
        out.r = new double[3*N];
        out.J = new double[3*N*3];

        for (int k = 1; k < Nfull; k++) {
            double sL = p.rL * d.dphiL[k];
            double sR = p.rR * d.dphiR[k];
            double ds = 0.5 * (sR + sL);
            double dth = (sR - sL) / p.b;

            double a = wrapAngle(d.thPrev[k] + 0.5 * dth);
            double ca = Math.cos(a);
            double sa = Math.sin(a);

            double dx = ds * ca;
            double dy = ds * sa;

            double rx = dx - d.dxGt[k];
            double ry = dy - d.dyGt[k];
            double rth = wrapAngle(dth - d.dthGt[k]);

            int i = k - 1;
            out.r[3*i + 0] = rx;
            out.r[3*i + 1] = ry;
            out.r[3*i + 2] = wTheta * rth;

            // Derivatives
            double ddsDrL = 0.5 * d.dphiL[k];
            double ddsDrR = 0.5 * d.dphiR[k];

            double ddthDrL = -(d.dphiL[k]) / p.b;
            double ddthDrR = (d.dphiR[k]) / p.b;
            double ddthDb  = -(sR - sL) / (p.b * p.b);

            double ddxDrL = ca * ddsDrL + ds * (-sa) * (0.5 * ddthDrL);
            double ddxDrR = ca * ddsDrR + ds * (-sa) * (0.5 * ddthDrR);
            double ddxDb  = ds * (-sa) * (0.5 * ddthDb);

            double ddyDrL = sa * ddsDrL + ds * ca * (0.5 * ddthDrL);
            double ddyDrR = sa * ddsDrR + ds * ca * (0.5 * ddthDrR);
            double ddyDb  = ds * ca * (0.5 * ddthDb);

            double ddthDrL_s = wTheta * ddthDrL;
            double ddthDrR_s = wTheta * ddthDrR;
            double ddthDb_s  = wTheta * ddthDb;

            // Fill J rows (row-major)
            int r0 = (3*i + 0)*3;
            int r1 = (3*i + 1)*3;
            int r2 = (3*i + 2)*3;

            out.J[r0 + 0] = ddxDrL; out.J[r0 + 1] = ddxDrR; out.J[r0 + 2] = ddxDb;
            out.J[r1 + 0] = ddyDrL; out.J[r1 + 1] = ddyDrR; out.J[r1 + 2] = ddyDb;
            out.J[r2 + 0] = ddthDrL_s; out.J[r2 + 1] = ddthDrR_s; out.J[r2 + 2] = ddthDb_s;
        }
        return out;
    }

    // Solve 3x3 with Gaussian elimination
    static boolean solve3x3(double[] A, double[] b, double[] x) {
        double[][] M = {
            {A[0], A[1], A[2], b[0]},
            {A[3], A[4], A[5], b[1]},
            {A[6], A[7], A[8], b[2]}
        };

        for (int i = 0; i < 3; i++) {
            int piv = i;
            for (int r = i+1; r < 3; r++) {
                if (Math.abs(M[r][i]) > Math.abs(M[piv][i])) piv = r;
            }
            if (Math.abs(M[piv][i]) < 1e-18) return false;
            if (piv != i) {
                double[] tmp = M[i]; M[i] = M[piv]; M[piv] = tmp;
            }
            double inv = 1.0 / M[i][i];
            for (int c = i; c < 4; c++) M[i][c] *= inv;
            for (int r = 0; r < 3; r++) {
                if (r == i) continue;
                double f = M[r][i];
                for (int c = i; c < 4; c++) M[r][c] -= f * M[i][c];
            }
        }
        x[0] = M[0][3]; x[1] = M[1][3]; x[2] = M[2][3];
        return true;
    }

    static Params gaussNewtonCalibrate(ArrayList<Row> allRows, double ticksPerRev, Params p0, double wTheta, int maxIter, double damping) {
        Params p = new Params(p0.rL, p0.rR, p0.b);
        for (int it = 0; it < maxIter; it++) {
            ResidualJac rj = stackResidualJac(allRows, ticksPerRev, p, wTheta);
            double[] r = rj.r;
            double[] J = rj.J;
            int m = r.length;

            double[] A = new double[9];
            double[] g = new double[3];

            for (int row = 0; row < m; row++) {
                int off = row*3;
                double j0 = J[off + 0], j1 = J[off + 1], j2 = J[off + 2];
                g[0] += j0 * r[row];
                g[1] += j1 * r[row];
                g[2] += j2 * r[row];
                A[0] += j0*j0; A[1] += j0*j1; A[2] += j0*j2;
                A[3] += j1*j0; A[4] += j1*j1; A[5] += j1*j2;
                A[6] += j2*j0; A[7] += j2*j1; A[8] += j2*j2;
            }
            A[0] += damping; A[4] += damping; A[8] += damping;

            double[] b = new double[]{-g[0], -g[1], -g[2]};
            double[] dp = new double[3];
            if (!solve3x3(A, b, dp)) break;

            double norm = Math.sqrt(dp[0]*dp[0] + dp[1]*dp[1] + dp[2]*dp[2]);
            if (norm < 1e-12) break;

            double alpha = 1.0;
            for (int ls = 0; ls < 10; ls++) {
                double nrL = p.rL + alpha*dp[0];
                double nrR = p.rR + alpha*dp[1];
                double nb  = p.b  + alpha*dp[2];
                if (nrL > 0 && nrR > 0 && nb > 0) {
                    p.rL = nrL; p.rR = nrR; p.b = nb;
                    break;
                }
                alpha *= 0.5;
            }
        }
        return p;
    }

    static Metrics computeMetricsReconstructed(ArrayList<Row> v, int trialId, double ticksPerRev, Params p) {
        int N = v.size();
        double[] dphiL = new double[N];
        double[] dphiR = new double[N];

        for (int k = 1; k < N; k++) {
            Row a = v.get(k-1);
            Row b = v.get(k);
            dphiL[k] = ticksToDphi(b.nL - a.nL, ticksPerRev);
            dphiR[k] = ticksToDphi(b.nR - a.nR, ticksPerRev);
        }

        // integrate from gt start
        double x = v.get(0).xG;
        double y = v.get(0).yG;
        double th = v.get(0).thG;

        double sumPos2 = 0.0;
        double sumTh2 = 0.0;

        for (int k = 0; k < N; k++) {
            if (k > 0) {
                double sL = p.rL * dphiL[k];
                double sR = p.rR * dphiR[k];
                double ds = 0.5 * (sR + sL);
                double dth = (sR - sL) / p.b;
                double thMid = wrapAngle(th + 0.5 * dth);

                x += ds * Math.cos(thMid);
                y += ds * Math.sin(thMid);
                th = wrapAngle(th + dth);
            }

            Row r = v.get(k);
            double ex = x - r.xG;
            double ey = y - r.yG;
            double eth = wrapAngle(th - r.thG);
            double epos = Math.sqrt(ex*ex + ey*ey);

            sumPos2 += epos*epos;
            sumTh2 += eth*eth;
        }

        Metrics m = new Metrics();
        m.trialId = trialId;
        m.rmsePos = Math.sqrt(sumPos2 / Math.max(1, N));
        m.rmseTh  = Math.sqrt(sumTh2 / Math.max(1, N));

        Row rN = v.get(N-1);
        double exN = x - rN.xG;
        double eyN = y - rN.yG;
        m.endPos = Math.sqrt(exN*exN + eyN*eyN);
        m.endTh  = Math.abs(wrapAngle(th - rN.thG));

        double s = 0.0;
        for (int k = 1; k < N; k++) {
            Row a = v.get(k-1);
            Row b = v.get(k);
            double dx = b.xG - a.xG;
            double dy = b.yG - a.yG;
            s += Math.sqrt(dx*dx + dy*dy);
        }
        m.sTotal = s;
        m.driftPerM = (s > 1e-12) ? (m.endPos / s) : Double.NaN;
        return m;
    }

    static void printMetricsTable(ArrayList<Metrics> ms) {
        System.out.println("trial_id, rmse_pos, rmse_th, end_pos, end_th, s_total, drift_per_m");
        for (Metrics m : ms) {
            System.out.printf(Locale.US,
                    "%d, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f%n",
                    m.trialId, m.rmsePos, m.rmseTh, m.endPos, m.endTh, m.sTotal, m.driftPerM);
        }
    }

    public static void main(String[] args) throws Exception {
        String csvPath = "";
        double ticksPerRev = 0.0;
        double rNom = 0.05;
        double bNom = 0.30;
        double wTheta = 1.0;

        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            if (a.equals("--csv")) csvPath = args[++i];
            else if (a.equals("--ticks_per_rev")) ticksPerRev = Double.parseDouble(args[++i]);
            else if (a.equals("--r_nom")) rNom = Double.parseDouble(args[++i]);
            else if (a.equals("--b_nom")) bNom = Double.parseDouble(args[++i]);
            else if (a.equals("--w_theta")) wTheta = Double.parseDouble(args[++i]);
        }

        if (csvPath.isEmpty() || ticksPerRev <= 0) {
            System.err.println("Usage: java Chapter5_Lesson5 --csv log.csv --ticks_per_rev 4096 [--r_nom 0.05 --b_nom 0.30 --w_theta 1.0]");
            System.exit(1);
        }

        ArrayList<Row> rows = loadCSV(csvPath);

        // group by trial
        HashMap<Integer, ArrayList<Row>> trials = new HashMap<>();
        for (Row r : rows) {
            trials.computeIfAbsent(r.trialId, k -> new ArrayList<>()).add(r);
        }

        ArrayList<Metrics> mLogged = new ArrayList<>();
        for (Map.Entry<Integer, ArrayList<Row>> kv : trials.entrySet()) {
            mLogged.add(computeMetricsLogged(kv.getValue(), kv.getKey()));
        }

        System.out.println("=== Metrics (logged odom) ===");
        printMetricsTable(mLogged);
        System.out.println();

        Params p0 = new Params(rNom, rNom, bNom);
        Params pHat = gaussNewtonCalibrate(rows, ticksPerRev, p0, wTheta, 15, 1e-9);

        System.out.println("=== Calibration Result ===");
        System.out.printf(Locale.US, "rL_hat = %.10f m%n", pHat.rL);
        System.out.printf(Locale.US, "rR_hat = %.10f m%n", pHat.rR);
        System.out.printf(Locale.US, "b_hat  = %.10f m%n", pHat.b);
        System.out.println();

        ArrayList<Metrics> mCal = new ArrayList<>();
        for (Map.Entry<Integer, ArrayList<Row>> kv : trials.entrySet()) {
            mCal.add(computeMetricsReconstructed(kv.getValue(), kv.getKey(), ticksPerRev, pHat));
        }

        System.out.println("=== Metrics (reconstructed, calibrated) ===");
        printMetricsTable(mCal);
        System.out.println();
    }
}
