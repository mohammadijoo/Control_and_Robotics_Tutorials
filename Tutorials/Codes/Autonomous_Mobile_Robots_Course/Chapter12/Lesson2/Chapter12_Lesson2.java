// Chapter12_Lesson2.java
// Loop Closure Detection Concepts — minimal Java demo (cosine retrieval + chi-square gate)
// Compile/run:
//   javac Chapter12_Lesson2.java
//   java Chapter12_Lesson2
//
// For a production system, use a proper nearest-neighbor index (e.g., KD-tree / HNSW)
// and a geometric verifier (RANSAC + scan matching / PnP).

import java.util.Random;

public class Chapter12_Lesson2 {

    static class Pose2 {
        double x, y, th;
        Pose2(double x, double y, double th) { this.x=x; this.y=y; this.th=th; }
    }

    static double wrap(double a) {
        while (a > Math.PI) a -= 2.0*Math.PI;
        while (a < -Math.PI) a += 2.0*Math.PI;
        return a;
    }

    static Pose2[] makeSquareLoop(int N, double side) {
        Pose2[] gt = new Pose2[N];
        double perim = 4.0*side;
        for (int i=0;i<N;i++) {
            double s = (perim * i) / N;
            int seg = (int)Math.floor(s / side);
            double u = s % side;
            double x,y,th;
            if (seg==0) { x=u; y=0.0; th=0.0; }
            else if (seg==1) { x=side; y=u; th=Math.PI/2; }
            else if (seg==2) { x=side-u; y=side; th=Math.PI; }
            else { x=0.0; y=side-u; th=-Math.PI/2; }
            gt[i] = new Pose2(x,y,th);
        }
        return gt;
    }

    static Pose2[] addOdometryDrift(Pose2[] gt, double sigmaXY, double sigmaTh,
                                   double driftX, double driftY, double driftTh, long seed) {
        int N = gt.length;
        Pose2[] odo = new Pose2[N];
        odo[0] = new Pose2(gt[0].x, gt[0].y, gt[0].th);

        Random rng = new Random(seed);
        for (int k=1;k<N;k++) {
            double dx = gt[k].x - gt[k-1].x;
            double dy = gt[k].y - gt[k-1].y;
            double dth = wrap(gt[k].th - gt[k-1].th);

            double incx = dx + sigmaXY * rng.nextGaussian() + driftX;
            double incy = dy + sigmaXY * rng.nextGaussian() + driftY;
            double inct = dth + sigmaTh * rng.nextGaussian() + driftTh;

            odo[k] = new Pose2(
                odo[k-1].x + incx,
                odo[k-1].y + incy,
                wrap(odo[k-1].th + inct)
            );
        }
        return odo;
    }

    static double dot(double[] a, double[] b) {
        double s=0.0;
        for (int i=0;i<a.length;i++) s += a[i]*b[i];
        return s;
    }

    static double norm(double[] a) {
        return Math.sqrt(dot(a,a));
    }

    static double cosine(double[] a, double[] b) {
        double na = norm(a), nb = norm(b);
        if (na < 1e-12 || nb < 1e-12) return 0.0;
        return dot(a,b)/(na*nb);
    }

    static int placeId(double x, double y, int nPlaces) {
        int qx = (int)Math.floor(x/3.2);
        int qy = (int)Math.floor(y/3.2);
        long h = 1L*qx*73856093L + 1L*qy*19349663L;
        long m = ((h % nPlaces) + nPlaces) % nPlaces;
        return (int)m;
    }

    // Hard-coded chi-square 0.995 quantile for df=3 (approx 12.838)
    static final double CHI2_3_0995 = 12.838;

    static boolean chiSquareGate(double[] r, double sigmaXY, double sigmaTh) {
        double d2 =
            (r[0]*r[0])/(sigmaXY*sigmaXY) +
            (r[1]*r[1])/(sigmaXY*sigmaXY) +
            (r[2]*r[2])/(sigmaTh*sigmaTh);
        return d2 < CHI2_3_0995;
    }

    public static void main(String[] args) {
        final int N = 420;
        final int V = 300;
        final int MIN_SEP = 40;
        final double SIDE = 25.0;

        Pose2[] gt = makeSquareLoop(N, SIDE);
        Pose2[] odo = addOdometryDrift(gt, 0.03, 0.003, 0.002, -0.001, 0.0002, 0L);

        // prototypes
        final int N_PLACES = 90;
        double[][] prot = new double[N_PLACES][V];
        Random rng = new Random(1L);

        for (int p=0;p<N_PLACES;p++) {
            for (int t=0;t<20;t++) {
                int idx = rng.nextInt(V);
                prot[p][idx] = 0.5 + 1.5*rng.nextDouble();
            }
            // normalize
            double np = norm(prot[p]);
            if (np > 1e-12) for (int v=0;v<V;v++) prot[p][v] /= np;
        }

        // descriptors (TF normalized)
        double[][] X = new double[N][V];
        for (int i=0;i<N;i++) {
            int pid = placeId(gt[i].x, gt[i].y, N_PLACES);
            double sum=0.0;
            for (int v=0;v<V;v++) {
                double lam = 8.0*prot[pid][v] + 0.15;
                // cheap Poisson approximation via Gaussian for speed
                double c = Math.max(0.0, lam + Math.sqrt(lam)*rng.nextGaussian());
                X[i][v] = c;
                sum += c;
            }
            if (sum > 1e-12) for (int v=0;v<V;v++) X[i][v] /= sum;
        }

        // ground-truth loops
        boolean[][] isLoop = new boolean[N][N];
        int gtCount = 0;
        for (int i=0;i<N;i++) {
            for (int j=0;j<i-MIN_SEP;j++) {
                double dx = gt[i].x - gt[j].x;
                double dy = gt[i].y - gt[j].y;
                if (Math.hypot(dx,dy) < 1.4) { isLoop[i][j] = true; gtCount++; }
            }
        }

        int det=0, tp=0, fp=0;
        for (int i=0;i<N;i++) {
            int jmax = i - MIN_SEP;
            if (jmax <= 0) continue;

            // best match by cosine
            double bestSim = -1.0;
            int bestJ = -1;
            for (int j=0;j<jmax;j++) {
                double sim = cosine(X[i], X[j]);
                if (sim > bestSim) { bestSim = sim; bestJ = j; }
            }
            if (bestSim < 0.75) continue;

            // geometric verification
            double sigmaXY = 0.20, sigmaTh = 0.07;

            double predx = odo[i].x - odo[bestJ].x;
            double predy = odo[i].y - odo[bestJ].y;
            double predt = wrap(odo[i].th - odo[bestJ].th);

            double zx = (gt[i].x - gt[bestJ].x) + sigmaXY*rng.nextGaussian();
            double zy = (gt[i].y - gt[bestJ].y) + sigmaXY*rng.nextGaussian();
            double zt = wrap((gt[i].th - gt[bestJ].th) + sigmaTh*rng.nextGaussian());

            double[] r = new double[] { zx - predx, zy - predy, wrap(zt - predt) };

            if (!chiSquareGate(r, sigmaXY, sigmaTh)) continue;

            det++;
            if (isLoop[i][bestJ]) tp++; else fp++;
        }

        double prec = tp / (double)(tp + fp + 1e-12);
        double rec  = tp / (double)(gtCount + 1e-12);

        System.out.println("N=" + N);
        System.out.println("GT loops=" + gtCount);
        System.out.println("Detected=" + det);
        System.out.println("Precision=" + String.format("%.3f", prec) +
                           "  Recall=" + String.format("%.3f", rec));
    }
}
