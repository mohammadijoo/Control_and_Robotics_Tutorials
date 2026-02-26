\
/* Chapter20_Lesson5.java
 * AMR Capstone final demo evaluation + research-style summary (Java 17)
 */
import java.util.*;

public class Chapter20_Lesson5 {
    static class Run {
        double success, time, path, ref, rms, clr, col, lat, en, iou;
        Run(double success, double time, double path, double ref, double rms, double clr, double col, double lat, double en, double iou) {
            this.success = success; this.time = time; this.path = path; this.ref = ref; this.rms = rms;
            this.clr = clr; this.col = col; this.lat = lat; this.en = en; this.iou = iou;
        }
    }
    static double mean(double[] x) { double s = 0; for (double v : x) s += v; return s / x.length; }
    static double stdev(double[] x) { double m = mean(x), s = 0; for (double v : x) s += (v - m)*(v - m); return Math.sqrt(s/(x.length-1)); }
    static double eff(Run r) { return r.ref / r.path; }
    static double enpm(Run r) { return r.en / r.path; }

    static class Agg { double succ, cfree, time, eff, rms, clr, lat, enpm, iou; }

    static Agg aggregate(List<Run> runs) {
        int n = runs.size();
        double[] succ = new double[n], cfree = new double[n], time = new double[n], ef = new double[n];
        double[] rms = new double[n], clr = new double[n], lat = new double[n], enpm = new double[n], iou = new double[n];
        for (int i = 0; i < n; i++) {
            Run r = runs.get(i);
            succ[i]=r.success; cfree[i]=(r.col==0?1:0); time[i]=r.time; ef[i]=eff(r); rms[i]=r.rms;
            clr[i]=r.clr; lat[i]=r.lat; enpm[i]=enpm(r); iou[i]=r.iou;
        }
        Agg a = new Agg();
        a.succ = mean(succ); a.cfree = mean(cfree); a.time = mean(time); a.eff = mean(ef);
        a.rms = mean(rms); a.clr = mean(clr); a.lat = mean(lat); a.enpm = mean(enpm); a.iou = mean(iou);
        return a;
    }

    static double[] pairedCI(double[] p, double[] b) {
        double[] d = new double[p.length];
        for (int i = 0; i < p.length; i++) d[i] = p[i] - b[i];
        double m = mean(d);
        double h = 1.959963984540054 * stdev(d) / Math.sqrt(d.length);
        return new double[]{m, m-h, m+h};
    }

    public static void main(String[] args) {
        List<Run> baseline = Arrays.asList(
            new Run(1,118.2,28.1,22.0,0.19,0.24,1,31.2,24.8,0.72),
            new Run(1,104.3,24.9,20.0,0.17,0.28,0,28.7,20.1,0.79),
            new Run(0,138.8,30.4,23.0,0.31,0.12,2,39.1,28.5,0.58),
            new Run(1,96.5,22.7,19.0,0.13,0.35,0,24.6,18.4,0.82),
            new Run(1,110.1,26.8,21.0,0.22,0.21,1,33.8,23.7,0.74)
        );
        List<Run> proposed = Arrays.asList(
            new Run(1,91.4,24.0,22.0,0.11,0.31,0,22.3,21.2,0.84),
            new Run(1,86.0,21.8,20.0,0.10,0.34,0,20.4,18.8,0.87),
            new Run(1,104.2,25.6,23.0,0.16,0.22,1,24.9,22.1,0.76),
            new Run(1,80.9,20.5,19.0,0.09,0.39,0,18.9,17.3,0.90),
            new Run(1,89.1,23.5,21.0,0.14,0.27,0,21.8,19.6,0.83)
        );
        Agg A = aggregate(baseline), B = aggregate(proposed);
        double[] tB = baseline.stream().mapToDouble(r -> r.time).toArray();
        double[] tP = proposed.stream().mapToDouble(r -> r.time).toArray();
        double[] iB = baseline.stream().mapToDouble(r -> r.iou).toArray();
        double[] iP = proposed.stream().mapToDouble(r -> r.iou).toArray();
        double[] ciTime = pairedCI(tP, tB);
        double[] ciIoU = pairedCI(iP, iB);

        System.out.printf(Locale.US, "Baseline success=%.4f, proposed success=%.4f%n", A.succ, B.succ);
        System.out.printf(Locale.US, "Baseline time=%.4f, proposed time=%.4f%n", A.time, B.time);
        System.out.printf(Locale.US, "Baseline eff=%.4f, proposed eff=%.4f%n", A.eff, B.eff);
        System.out.printf(Locale.US, "Baseline IoU=%.4f, proposed IoU=%.4f%n", A.iou, B.iou);
        System.out.printf(Locale.US, "Paired delta time: %.4f [%.4f, %.4f]%n", ciTime[0], ciTime[1], ciTime[2]);
        System.out.printf(Locale.US, "Paired delta IoU: %.4f [%.4f, %.4f]%n", ciIoU[0], ciIoU[1], ciIoU[2]);
    }
}
