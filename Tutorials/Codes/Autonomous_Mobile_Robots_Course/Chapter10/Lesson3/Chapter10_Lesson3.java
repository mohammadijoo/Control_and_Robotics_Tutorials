// Chapter10_Lesson3.java
/*
Correlative Scan Matching (2D) — minimal Java reference implementation.

This is a didactic, self-contained example:
  - occupancy grid (binary) + simple softened likelihood field,
  - discrete coarse-to-fine search for (x,y,theta) maximizing correlation.

For production robotics stacks, you would typically integrate with:
  - ROS2 (Java via rcljava) or custom middleware,
  - a high-quality map representation (log-odds grid / submaps),
  - branch-and-bound fast correlative search.

No external libraries required (only standard Java).
*/

import java.util.*;

public class Chapter10_Lesson3 {

    static class Pose2 {
        double x, y, th;
        Pose2(double x, double y, double th){ this.x=x; this.y=y; this.th=th; }
    }

    static class Cand {
        Pose2 p; double score;
        Cand(Pose2 p, double s){ this.p=p; this.score=s; }
    }

    static int idx(int r, int c, int W){ return r*W + c; }

    static byte[] makeOcc(int H, int W, double res){
        byte[] occ = new byte[H*W];
        // walls
        for(int c=0;c<W;c++){ occ[idx(0,c,W)] = 1; occ[idx(H-1,c,W)] = 1; }
        for(int r=0;r<H;r++){ occ[idx(r,0,W)] = 1; occ[idx(r,W-1,W)] = 1; }
        // pillar
        int cx=W/2, cy=H/2;
        int rad = (int)Math.round(0.4 / res);
        for(int r=cy-rad; r<=cy+rad; r++){
            for(int c=cx-rad; c<=cx+rad; c++){
                if (r>=0 && r<H && c>=0 && c<W) occ[idx(r,c,W)] = 1;
            }
        }
        return occ;
    }

    static float[] softenToField(byte[] occ, int H, int W){
        float[] field = new float[H*W];
        int rad = 4;
        for(int r=0;r<H;r++){
            for(int c=0;c<W;c++){
                float best = 0f;
                for(int dr=-rad; dr<=rad; dr++){
                    for(int dc=-rad; dc<=rad; dc++){
                        int rr=r+dr, cc=c+dc;
                        if (rr<0||rr>=H||cc<0||cc>=W) continue;
                        if (occ[idx(rr,cc,W)] != 0){
                            float d2 = (float)(dr*dr + dc*dc);
                            float s = (float)Math.exp(-d2 / (2.0*rad*rad));
                            if (s>best) best=s;
                        }
                    }
                }
                field[idx(r,c,W)] = best;
            }
        }
        return field;
    }

    static float[] downsampleMax(float[] g, int H, int W, int[] outHW){
        int H2 = H/2, W2 = W/2;
        float[] out = new float[H2*W2];
        for(int r=0;r<H2;r++){
            for(int c=0;c<W2;c++){
                float m = 0f;
                m = Math.max(m, g[idx(2*r,2*c,W)]);
                m = Math.max(m, g[idx(2*r+1,2*c,W)]);
                m = Math.max(m, g[idx(2*r,2*c+1,W)]);
                m = Math.max(m, g[idx(2*r+1,2*c+1,W)]);
                out[idx(r,c,W2)] = m;
            }
        }
        outHW[0]=H2; outHW[1]=W2;
        return out;
    }

    static double scorePose(float[] field, int H, int W, double res, Pose2 p, double[][] pts_r){
        double c = Math.cos(p.th), s = Math.sin(p.th);
        double sum = 0.0;
        for(int i=0;i<pts_r.length;i++){
            double xr = pts_r[i][0], yr = pts_r[i][1];
            double xw = c*xr - s*yr + p.x;
            double yw = s*xr + c*yr + p.y;
            int gx = (int)Math.floor(xw / res);
            int gy = (int)Math.floor(yw / res);
            if (gx>=0 && gx<W && gy>=0 && gy<H){
                sum += field[idx(gy,gx,W)];
            }
        }
        return sum;
    }

    static List<Cand> searchLevel(float[] field, int H, int W, double res,
                                 List<Cand> seeds, double[][] pts_r,
                                 double winXY, double winTh, double stepXY, double stepTh, int topK){
        ArrayList<Cand> cands = new ArrayList<>();
        for(Cand seed : seeds){
            for(double x = seed.p.x - winXY; x <= seed.p.x + winXY + 1e-12; x += stepXY){
                for(double y = seed.p.y - winXY; y <= seed.p.y + winXY + 1e-12; y += stepXY){
                    for(double th = seed.p.th - winTh; th <= seed.p.th + winTh + 1e-12; th += stepTh){
                        Pose2 p = new Pose2(x,y,th);
                        double sc = scorePose(field, H, W, res, p, pts_r);
                        cands.add(new Cand(p, sc));
                    }
                }
            }
        }
        cands.sort((a,b) -> Double.compare(b.score, a.score));
        if (cands.size() > topK) return cands.subList(0, topK);
        return cands;
    }

    public static void main(String[] args){
        double res = 0.05;
        double sizeM = 10.0;
        int W = (int)Math.round(sizeM / res);
        int H = W;

        byte[] occ = makeOcc(H,W,res);
        float[] field0 = softenToField(occ,H,W);
        int[] hw1 = new int[2]; int[] hw2 = new int[2];
        float[] field1 = downsampleMax(field0,H,W,hw1);
        float[] field2 = downsampleMax(field1,hw1[0],hw1[1],hw2);

        // true pose and synthetic scan (occupied points in range; no raycast)
        Pose2 ptrue = new Pose2(2.4, 3.2, Math.toRadians(18.0));

        ArrayList<double[]> occPtsW = new ArrayList<>();
        for(int r=0;r<H;r+=2){
            for(int c=0;c<W;c+=2){
                if (occ[idx(r,c,W)] != 0){
                    occPtsW.add(new double[]{(c+0.5)*res, (r+0.5)*res});
                }
            }
        }

        double ct = Math.cos(ptrue.th), st = Math.sin(ptrue.th);
        Random rng = new Random(7);
        ArrayList<double[]> ptsR = new ArrayList<>();
        for(double[] pw : occPtsW){
            double dx = pw[0]-ptrue.x, dy = pw[1]-ptrue.y;
            double xr =  ct*dx + st*dy;
            double yr = -st*dx + ct*dy;
            double d = Math.hypot(xr, yr);
            if (d>0.3 && d<6.0){
                // small Gaussian noise
                double nx = xr + rng.nextGaussian()*0.01;
                double ny = yr + rng.nextGaussian()*0.01;
                ptsR.add(new double[]{nx, ny});
            }
        }
        Collections.shuffle(ptsR, rng);
        if (ptsR.size() > 600) ptsR = new ArrayList<>(ptsR.subList(0, 600));
        double[][] pts_r = ptsR.toArray(new double[0][0]);

        Pose2 p0 = new Pose2(2.55, 3.05, Math.toRadians(10.0));
        List<Cand> seeds = new ArrayList<>();
        seeds.add(new Cand(p0, 0.0));

        seeds = searchLevel(field2, hw2[0], hw2[1], res*4.0, seeds, pts_r,
                            0.6, Math.toRadians(12), res*4.0, Math.toRadians(2.0), 50);
        seeds = searchLevel(field1, hw1[0], hw1[1], res*2.0, seeds, pts_r,
                            0.3, Math.toRadians(6), res*2.0, Math.toRadians(1.0), 50);
        seeds = searchLevel(field0, H, W, res, seeds, pts_r,
                            0.15, Math.toRadians(3), res, Math.toRadians(0.5), 50);

        Cand best = seeds.get(0);
        for(Cand c : seeds) if (c.score > best.score) best = c;

        System.out.printf("True pose:     x=%.3f y=%.3f th(deg)=%.2f%n", ptrue.x, ptrue.y, Math.toDegrees(ptrue.th));
        System.out.printf("Initial guess: x=%.3f y=%.3f th(deg)=%.2f%n", p0.x, p0.y, Math.toDegrees(p0.th));
        System.out.printf("Matched pose:  x=%.3f y=%.3f th(deg)=%.2f score=%.2f%n",
                best.p.x, best.p.y, Math.toDegrees(best.p.th), best.score);
    }
}
