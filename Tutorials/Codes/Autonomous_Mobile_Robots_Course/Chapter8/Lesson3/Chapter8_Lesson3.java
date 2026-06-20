
/*
Chapter8_Lesson3.java

Autonomous Mobile Robots (Control Engineering) - Chapter 8, Lesson 3
Sensor Likelihoods for LiDAR and Vision

This file provides:
1) A 2D occupancy grid and approximate Euclidean distance transform (multi-source Dijkstra, 8-neighbors).
2) A LiDAR likelihood-field model.
3) A bearing-only vision landmark likelihood with an outlier mixture.
4) Stable log-weight normalization.

Robotics ecosystem notes:
- Java robotics stacks often use OpenCV Java bindings for vision, and (legacy) rosjava for ROS1.
- For ROS2, Java client libraries exist but are less common; the likelihood logic here is framework-agnostic.

Compile/Run:
  javac Chapter8_Lesson3.java
  java Chapter8_Lesson3
*/
import java.util.*;

public class Chapter8_Lesson3 {

    static class Pose2D {
        double x, y, theta;
        Pose2D(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
    }

    static double wrapToPi(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    static class OccupancyGrid2D {
        final int w, h;
        final double resolution;
        final double originX, originY;
        final byte[] occ; // 1 obstacle, 0 free

        OccupancyGrid2D(int w, int h, double resolution, double originX, double originY) {
            this.w = w; this.h = h;
            this.resolution = resolution;
            this.originX = originX; this.originY = originY;
            this.occ = new byte[w*h];
        }

        boolean inBounds(int gx, int gy) { return (0 <= gx && gx < w && 0 <= gy && gy < h); }
        int idx(int gx, int gy) { return gy*w + gx; }

        void setOcc(int gx, int gy, int v) { occ[idx(gx,gy)] = (byte)v; }
        int  getOcc(int gx, int gy) { return occ[idx(gx,gy)]; }

        int[] worldToGrid(double x, double y) {
            int gx = (int)Math.floor((x - originX) / resolution);
            int gy = (int)Math.floor((y - originY) / resolution);
            return new int[]{gx, gy};
        }
    }

    static float[] distanceTransformBrushfire(OccupancyGrid2D grid) {
        final int W = grid.w, H = grid.h;
        final float INF = Float.POSITIVE_INFINITY;
        float[] dist = new float[W*H];
        Arrays.fill(dist, INF);

        class Node {
            float d; int gx, gy;
            Node(float d, int gx, int gy) { this.d = d; this.gx = gx; this.gy = gy; }
        }

        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(n -> n.d));

        for (int gy = 0; gy < H; gy++) {
            for (int gx = 0; gx < W; gx++) {
                if (grid.getOcc(gx, gy) != 0) {
                    int id = grid.idx(gx, gy);
                    dist[id] = 0.0f;
                    pq.add(new Node(0.0f, gx, gy));
                }
            }
        }

        int[][] nbh = new int[][]{
                {-1,0},{1,0},{0,-1},{0,1},
                {-1,-1},{-1,1},{1,-1},{1,1}
        };

        while (!pq.isEmpty()) {
            Node cur = pq.poll();
            int id = grid.idx(cur.gx, cur.gy);
            if (cur.d > dist[id]) continue;

            for (int[] dxy : nbh) {
                int ngx = cur.gx + dxy[0];
                int ngy = cur.gy + dxy[1];
                if (!grid.inBounds(ngx, ngy)) continue;
                float step = (dxy[0] == 0 || dxy[1] == 0) ? 1.0f : (float)Math.sqrt(2.0);
                float nd = cur.d + step;
                int jd = grid.idx(ngx, ngy);
                if (nd < dist[jd]) {
                    dist[jd] = nd;
                    pq.add(new Node(nd, ngx, ngy));
                }
            }
        }

        for (int i = 0; i < dist.length; i++) dist[i] *= (float)grid.resolution;
        return dist;
    }

    static class LikelihoodFieldModel {
        final OccupancyGrid2D grid;
        final float[] distFieldM;
        final double zMax, sigmaHit, wHit, wRand, maxDist;

        LikelihoodFieldModel(OccupancyGrid2D grid, float[] distFieldM,
                             double zMax, double sigmaHit,
                             double wHit, double wRand, double maxDist) {
            this.grid = grid; this.distFieldM = distFieldM;
            this.zMax = zMax; this.sigmaHit = sigmaHit;
            this.wHit = wHit; this.wRand = wRand; this.maxDist = maxDist;
        }

        double endpointDistance(double wx, double wy) {
            int[] g = grid.worldToGrid(wx, wy);
            int gx = g[0], gy = g[1];
            if (!grid.inBounds(gx, gy)) return maxDist;
            float d = distFieldM[grid.idx(gx, gy)];
            return Math.min(maxDist, (double)d);
        }

        double logLikelihood(Pose2D pose, double[] ranges, double[] relAngles) {
            int n = Math.min(ranges.length, relAngles.length);
            double invZ = 1.0 / zMax;
            double sig2 = sigmaHit * sigmaHit;
            double logp = 0.0;

            for (int i = 0; i < n; i++) {
                double z = Math.max(0.0, Math.min(ranges[i], zMax));
                double a = pose.theta + relAngles[i];
                double wx = pose.x + z * Math.cos(a);
                double wy = pose.y + z * Math.sin(a);

                double d = endpointDistance(wx, wy);
                double pHit = Math.exp(-(d*d) / (2.0*sig2));
                double p = wHit * pHit + wRand * invZ;
                logp += Math.log(Math.max(p, 1e-12));
            }
            return logp;
        }
    }

    static class VisionBearingModel {
        final Map<Integer, double[]> landmarks; // id -> [x,y]
        final double sigmaBearing, epsOutlier;

        VisionBearingModel(Map<Integer, double[]> landmarks, double sigmaBearing, double epsOutlier) {
            this.landmarks = landmarks;
            this.sigmaBearing = sigmaBearing;
            this.epsOutlier = epsOutlier;
        }

        double logLikelihood(Pose2D pose, int[] ids, double[] bearings) {
            int n = Math.min(ids.length, bearings.length);
            double sig2 = sigmaBearing * sigmaBearing;
            double norm = 1.0 / Math.sqrt(2.0 * Math.PI * sig2);
            double uni  = 1.0 / (2.0 * Math.PI);

            double logp = 0.0;
            for (int i = 0; i < n; i++) {
                int id = ids[i];
                double meas = bearings[i];
                double[] lm = landmarks.get(id);
                if (lm == null) {
                    logp += Math.log(uni);
                    continue;
                }
                double lx = lm[0], ly = lm[1];
                double pred = wrapToPi(Math.atan2(ly - pose.y, lx - pose.x) - pose.theta);
                double innov = wrapToPi(meas - pred);

                double pIn = norm * Math.exp(-(innov*innov) / (2.0*sig2));
                double p = (1.0 - epsOutlier) * pIn + epsOutlier * uni;
                logp += Math.log(Math.max(p, 1e-15));
            }
            return logp;
        }
    }

    static double[] normalizeLogWeights(double[] logw) {
        double m = logw[0];
        for (double v : logw) m = Math.max(m, v);
        double s = 0.0;
        for (double v : logw) s += Math.exp(v - m);
        double lse = m + Math.log(s);

        double[] w = new double[logw.length];
        double sumw = 0.0;
        for (int i = 0; i < logw.length; i++) {
            w[i] = Math.exp(logw[i] - lse);
            sumw += w[i];
        }
        for (int i = 0; i < w.length; i++) w[i] /= sumw;
        return w;
    }

    public static void main(String[] args) {
        // Create a synthetic map (wall + pillar)
        int W = 120, H = 120;
        OccupancyGrid2D grid = new OccupancyGrid2D(W, H, 0.05, -3.0, -3.0);
        for (int gx = 10; gx < 110; gx++) grid.setOcc(gx, 60, 1);
        for (int gy = 20; gy < 30; gy++)
            for (int gx = 85; gx < 95; gx++)
                grid.setOcc(gx, gy, 1);

        float[] dist = distanceTransformBrushfire(grid);
        LikelihoodFieldModel lidar = new LikelihoodFieldModel(grid, dist, 8.0, 0.15, 0.95, 0.05, 2.0);

        Map<Integer, double[]> landmarks = new HashMap<>();
        landmarks.put(1, new double[]{-1.0, 0.0});
        landmarks.put(2, new double[]{ 1.0, 0.5});
        landmarks.put(3, new double[]{ 0.0,-1.0});
        VisionBearingModel vision = new VisionBearingModel(landmarks, Math.toRadians(3.0), 0.10);

        Pose2D[] particles = new Pose2D[]{
                new Pose2D(-0.5, -0.8, Math.toRadians(90.0)),
                new Pose2D(-0.2, -0.6, Math.toRadians(88.0)),
                new Pose2D( 0.8,  0.1, Math.toRadians(30.0))
        };

        // Synthetic scan: 9 beams
        double[] relAngles = new double[9];
        for (int i = 0; i < 9; i++) relAngles[i] = Math.toRadians(-40.0 + 80.0 * i / 8.0);
        double[] ranges = new double[]{2.8, 3.0, 3.2, 3.1, 3.0, 3.2, 2.9, 2.7, 2.6};

        // Vision observations: (id, bearing)
        int[] ids = new int[]{1,2,3};
        double[] bearings = new double[]{Math.toRadians(70.0), Math.toRadians(20.0), Math.toRadians(-95.0)};

        double[] logw = new double[particles.length];
        for (int i = 0; i < particles.length; i++) {
            double llL = lidar.logLikelihood(particles[i], ranges, relAngles);
            double llV = vision.logLikelihood(particles[i], ids, bearings);
            logw[i] = llL + llV;
        }

        double[] w = normalizeLogWeights(logw);
        System.out.println("log-weights: " + Arrays.toString(logw));
        System.out.println("weights:     " + Arrays.toString(w));
    }
}
      