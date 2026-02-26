// Chapter17_Lesson5.java
/*
Autonomous Mobile Robots — Chapter 17 (Exploration and Active Mapping)
Lesson 5 Lab: Autonomous Exploration in Unknown Map (single robot)

Minimal Java grid exploration demo:
- Occupancy belief (log-odds)
- Frontier detection (unknown adjacent to free)
- Candidate goal scoring with IG - cost - risk

Compile:
  javac Chapter17_Lesson5.java
Run:
  java Chapter17_Lesson5
*/

import java.util.*;
import java.lang.Math;

public class Chapter17_Lesson5 {
    static double clamp(double x, double lo, double hi) { return Math.max(lo, Math.min(hi, x)); }
    static double logit(double p) {
        p = clamp(p, 1e-6, 1.0 - 1e-6);
        return Math.log(p / (1.0 - p));
    }
    static double sigmoid(double l) {
        if (l >= 0.0) {
            double z = Math.exp(-l);
            return 1.0 / (1.0 + z);
        } else {
            double z = Math.exp(l);
            return z / (1.0 + z);
        }
    }
    static double bernoulliEntropy(double p) {
        p = clamp(p, 1e-12, 1.0 - 1e-12);
        return -p * Math.log(p) - (1.0 - p) * Math.log(1.0 - p);
    }

    static class Pose2D { int x, y; Pose2D(int x, int y){ this.x=x; this.y=y; } }

    static class BeliefGrid {
        int w, h;
        double[] l;
        double lOcc, lFree, lMin, lMax;

        BeliefGrid(int w, int h, double pOcc, double pFree){
            this.w = w; this.h = h;
            this.l = new double[w*h];
            this.lOcc = logit(pOcc);
            this.lFree = logit(pFree);
            this.lMin = logit(0.02);
            this.lMax = logit(0.98);
        }
        int idx(int x,int y){ return y*w + x; }

        int classify(int x,int y){
            double p = sigmoid(l[idx(x,y)]);
            if (p >= 0.65) return 1;
            if (p <= 0.35) return 0;
            return -1;
        }
        double prob(int x,int y){ return sigmoid(l[idx(x,y)]); }

        void updateFree(int x,int y){
            int i = idx(x,y);
            l[i] = clamp(l[i] + lFree, lMin, lMax);
        }
        void updateOcc(int x,int y){
            int i = idx(x,y);
            l[i] = clamp(l[i] + lOcc, lMin, lMax);
        }

        double totalEntropy(){
            double s = 0.0;
            for(double li : l){
                double p = sigmoid(li);
                s += bernoulliEntropy(p);
            }
            return s;
        }
    }

    static class World {
        int w, h;
        byte[] occ;
        Random rng;

        World(int w, int h, double obstacleProb, long seed){
            this.w=w; this.h=h;
            this.occ = new byte[w*h];
            this.rng = new Random(seed);
            for(int y=0;y<h;y++){
                for(int x=0;x<w;x++){
                    occ[y*w+x] = (rng.nextDouble() < obstacleProb) ? (byte)1 : (byte)0;
                }
            }
            for(int x=0;x<w;x++){ occ[x]=1; occ[(h-1)*w+x]=1; }
            for(int y=0;y<h;y++){ occ[y*w]=1; occ[y*w+(w-1)]=1; }
        }

        boolean isOcc(int x,int y){ return occ[y*w+x] != 0; }

        static ArrayList<int[]> bresenham(int x0,int y0,int x1,int y1){
            ArrayList<int[]> cells = new ArrayList<>();
            int dx = Math.abs(x1-x0);
            int dy = -Math.abs(y1-y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx + dy;
            int x=x0, y=y0;
            while(true){
                cells.add(new int[]{x,y});
                if(x==x1 && y==y1) break;
                int e2 = 2*err;
                if(e2 >= dy){ err += dy; x += sx; }
                if(e2 <= dx){ err += dx; y += sy; }
            }
            return cells;
        }

        void senseUpdate(Pose2D pose, BeliefGrid belief, int nRays, int rMax){
            double fov = 2.0*Math.PI;
            double start = -fov/2.0;
            for(int k=0;k<nRays;k++){
                double theta = start + (double)k / Math.max(1, nRays-1) * fov;
                int x1 = (int)Math.round(pose.x + rMax*Math.cos(theta));
                int y1 = (int)Math.round(pose.y + rMax*Math.sin(theta));
                x1 = (int)clamp(x1, 0, w-1);
                y1 = (int)clamp(y1, 0, h-1);
                ArrayList<int[]> line = bresenham(pose.x, pose.y, x1, y1);
                for(int i=1;i<line.size();i++){
                    int cx = line.get(i)[0], cy = line.get(i)[1];
                    if(isOcc(cx,cy)){ belief.updateOcc(cx,cy); break; }
                    belief.updateFree(cx,cy);
                }
            }
        }
    }

    static int[][] neigh4(int x,int y){
        return new int[][]{{x+1,y},{x-1,y},{x,y+1},{x,y-1}};
    }

    static ArrayList<int[]> frontierCells(BeliefGrid belief){
        ArrayList<int[]> fronts = new ArrayList<>();
        for(int y=1;y<belief.h-1;y++){
            for(int x=1;x<belief.w-1;x++){
                if(belief.classify(x,y) != -1) continue;
                for(int[] nb : neigh4(x,y)){
                    if(belief.classify(nb[0], nb[1]) == 0){ fronts.add(new int[]{x,y}); break; }
                }
            }
        }
        return fronts;
    }

    static double approxIG(BeliefGrid belief, Pose2D pose, int rMax){
        int unknown = 0;
        int r2 = rMax*rMax;
        for(int y=Math.max(0, pose.y-rMax); y<Math.min(belief.h, pose.y+rMax+1); y++){
            for(int x=Math.max(0, pose.x-rMax); x<Math.min(belief.w, pose.x+rMax+1); x++){
                int dx=x-pose.x, dy=y-pose.y;
                if(dx*dx + dy*dy <= r2){
                    if(belief.classify(x,y) == -1) unknown++;
                }
            }
        }
        return unknown * Math.log(2.0);
    }

    static class PQNode {
        double d; int x,y;
        PQNode(double d,int x,int y){ this.d=d; this.x=x; this.y=y; }
    }

    static double dijkstraCost(BeliefGrid belief, int sx,int sy,int gx,int gy, int[] parent){
        int W=belief.w, H=belief.h;
        Arrays.fill(parent, -1);
        double[] dist = new double[W*H];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        dist[sy*W+sx] = 0.0;

        PriorityQueue<PQNode> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a.d));
        pq.add(new PQNode(0.0, sx, sy));

        while(!pq.isEmpty()){
            PQNode cur = pq.poll();
            int idx = cur.y*W + cur.x;
            if(cur.d != dist[idx]) continue;
            if(cur.x==gx && cur.y==gy) return cur.d;
            for(int[] nb : neigh4(cur.x, cur.y)){
                int nx=nb[0], ny=nb[1];
                if(nx<0||nx>=W||ny<0||ny>=H) continue;
                if(belief.classify(nx,ny)==1) continue;
                double step = (belief.classify(nx,ny)==0) ? 1.0 : 3.0;
                double nd = cur.d + step;
                int j = ny*W + nx;
                if(nd < dist[j]){
                    dist[j]=nd;
                    parent[j]=idx;
                    pq.add(new PQNode(nd,nx,ny));
                }
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    static ArrayList<int[]> recoverPath(BeliefGrid belief, int[] parent, int sx,int sy,int gx,int gy){
        int W=belief.w;
        int s = sy*W+sx;
        int g = gy*W+gx;
        ArrayList<int[]> path = new ArrayList<>();
        if(s==g){ path.add(new int[]{sx,sy}); return path; }
        if(parent[g] < 0) return path;
        int cur = g;
        while(cur != s && cur >= 0){
            int x = cur % W;
            int y = cur / W;
            path.add(new int[]{x,y});
            cur = parent[cur];
        }
        path.add(new int[]{sx,sy});
        Collections.reverse(path);
        return path;
    }

    static double pathRisk(BeliefGrid belief, ArrayList<int[]> path){
        if(path.isEmpty()) return Double.POSITIVE_INFINITY;
        double s=0.0;
        for(int[] c : path) s += belief.prob(c[0], c[1]);
        return s / (double)path.size();
    }

    public static void main(String[] args){
        int W=60, H=45;
        World world = new World(W,H,0.22,7L);
        BeliefGrid belief = new BeliefGrid(W,H,0.72,0.28);

        Pose2D pose = new Pose2D(2,2);
        outer:
        for(int y=1;y<H-1;y++){
            for(int x=1;x<W-1;x++){
                if(!world.isOcc(x,y)){ pose = new Pose2D(x,y); break outer; }
            }
        }

        double wIG=1.0, wCost=0.35, wRisk=1.2;
        int nRays=48, rMax=10;

        world.senseUpdate(pose, belief, nRays, rMax);

        double travel=0.0;
        int steps=0;

        int[] parent = new int[W*H];

        for(int it=0; it<2500; it++){
            steps++;
            ArrayList<int[]> fronts = frontierCells(belief);
            if(fronts.isEmpty()) break;
            Collections.shuffle(fronts, new Random(steps*17L + 3L));
            int K = Math.min(fronts.size(), 60);

            double bestScore = -1e18;
            int bestGX = fronts.get(0)[0], bestGY = fronts.get(0)[1];

            for(int i=0;i<K;i++){
                int gx = fronts.get(i)[0], gy = fronts.get(i)[1];
                double cost = dijkstraCost(belief, pose.x, pose.y, gx, gy, parent);
                if(!Double.isFinite(cost)) continue;
                ArrayList<int[]> path = recoverPath(belief, parent, pose.x, pose.y, gx, gy);
                if(path.size() < 2) continue;

                double ig = approxIG(belief, new Pose2D(gx,gy), rMax);
                double risk = pathRisk(belief, path);
                double score = wIG*ig - wCost*cost - wRisk*risk;
                if(score > bestScore){
                    bestScore = score;
                    bestGX = gx; bestGY = gy;
                }
            }

            dijkstraCost(belief, pose.x, pose.y, bestGX, bestGY, parent);
            ArrayList<int[]> path = recoverPath(belief, parent, pose.x, pose.y, bestGX, bestGY);
            if(path.size() < 2) break;
            int nx = path.get(1)[0], ny = path.get(1)[1];

            if(world.isOcc(nx,ny)){
                belief.updateOcc(nx,ny);
                continue;
            }
            pose = new Pose2D(nx,ny);
            travel += 1.0;
            world.senseUpdate(pose, belief, nRays, rMax);
        }

        int known=0;
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                if(belief.classify(x,y) != -1) known++;
            }
        }
        double knownFrac = (double)known / (double)(W*H);

        System.out.println("=== Exploration Summary (Java) ===");
        System.out.println("steps: " + steps);
        System.out.println("travel: " + travel);
        System.out.println("known_fraction: " + knownFrac);
        System.out.println("entropy_final: " + belief.totalEntropy());
    }
}
