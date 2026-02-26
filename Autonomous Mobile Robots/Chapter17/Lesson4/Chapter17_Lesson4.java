// Chapter17_Lesson4.java
// Exploration Under Limited Battery/Time (Budget-Aware Frontier Exploration)
// Self-contained demo without ROS. Designed for clarity, not maximum performance.
// Compile: javac Chapter17_Lesson4.java
// Run:     java Chapter17_Lesson4
import java.util.*;

public class Chapter17_Lesson4 {

    static class Budget { double E, T; Budget(double E,double T){this.E=E;this.T=T;} }
    static class RobotParams { double EStep=1.2, TStep=1.0, EBase=0.10, EReserve=15.0; }
    static class SensorParams { int r=3; }

    static double clamp(double x,double lo,double hi){ return Math.max(lo, Math.min(hi, x)); }
    static double entropyBern(double p){
        p = clamp(p, 1e-9, 1.0-1e-9);
        return -(p*Math.log(p) + (1.0-p)*Math.log(1.0-p));
    }

    static class Grid {
        int W,H;
        double[] p; // row-major
        Grid(int W,int H){ this.W=W; this.H=H; p=new double[W*H]; Arrays.fill(p, 0.5); }
        boolean inb(int x,int y){ return 0<=x && x<W && 0<=y && y<H; }
        int idx(int x,int y){ return y*W+x; }
        double get(int x,int y){ return p[idx(x,y)]; }
        void set(int x,int y,double v){ p[idx(x,y)] = clamp(v, 0.0, 1.0); }
        boolean isFree(int x,int y,double thr){ return get(x,y) < thr; }
        boolean isOcc(int x,int y,double thr){ return get(x,y) > thr; }
    }

    static class Cell {
        int x,y;
        Cell(int x,int y){ this.x=x; this.y=y; }
        @Override public boolean equals(Object o){
            if(!(o instanceof Cell)) return false;
            Cell c=(Cell)o; return x==c.x && y==c.y;
        }
        @Override public int hashCode(){ return Objects.hash(x,y); }
    }

    static List<Cell> neigh4(Grid g, Cell c){
        int x=c.x,y=c.y;
        int[][] cand={{x+1,y},{x-1,y},{x,y+1},{x,y-1}};
        ArrayList<Cell> out=new ArrayList<>();
        for(int[] q:cand) if(g.inb(q[0],q[1])) out.add(new Cell(q[0],q[1]));
        return out;
    }

    static List<Cell> astar(Grid g, Cell s, Cell goal, double occThr){
        if(!g.inb(s.x,s.y) || !g.inb(goal.x,goal.y)) return Collections.emptyList();
        if(!g.isFree(s.x,s.y,occThr) || !g.isFree(goal.x,goal.y,occThr)) return Collections.emptyList();

        class Node { double f; Cell c; Node(double f,Cell c){this.f=f;this.c=c;} }
        PriorityQueue<Node> pq=new PriorityQueue<>(Comparator.comparingDouble(a->a.f));
        Map<Cell, Double> gscore=new HashMap<>();
        Map<Cell, Cell> parent=new HashMap<>();

        java.util.function.ToDoubleFunction<Cell> h = (Cell c) -> Math.abs(c.x-goal.x)+Math.abs(c.y-goal.y);

        pq.add(new Node(h.applyAsDouble(s), s));
        gscore.put(s, 0.0);

        while(!pq.isEmpty()){
            Cell cur=pq.poll().c;
            if(cur.equals(goal)){
                ArrayList<Cell> path=new ArrayList<>();
                path.add(cur);
                while(parent.containsKey(cur)){
                    cur = parent.get(cur);
                    path.add(cur);
                }
                Collections.reverse(path);
                return path;
            }
            double gc = gscore.get(cur);
            for(Cell nb: neigh4(g, cur)){
                if(!g.isFree(nb.x, nb.y, occThr)) continue;
                double tent = gc + 1.0;
                if(!gscore.containsKey(nb) || tent < gscore.get(nb)){
                    gscore.put(nb, tent);
                    parent.put(nb, cur);
                    pq.add(new Node(tent + h.applyAsDouble(nb), nb));
                }
            }
        }
        return Collections.emptyList();
    }

    static List<Cell> detectFrontiers(Grid g, double freeThr){
        ArrayList<Cell> fr=new ArrayList<>();
        for(int y=0;y<g.H;y++){
            for(int x=0;x<g.W;x++){
                if(g.get(x,y) >= freeThr) continue;
                Cell c=new Cell(x,y);
                for(Cell nb: neigh4(g,c)){
                    double pn = g.get(nb.x, nb.y);
                    if(Math.abs(pn-0.5) < 0.15){ fr.add(c); break; }
                }
            }
        }
        return fr;
    }

    static double expectedIG(Grid belief, Cell pose, SensorParams sp){
        double ig=0.0;
        for(int dy=-sp.r; dy<=sp.r; dy++){
            for(int dx=-sp.r; dx<=sp.r; dx++){
                int x=pose.x+dx, y=pose.y+dy;
                if(!belief.inb(x,y)) continue;
                double p0=belief.get(x,y);
                double H0=entropyBern(p0);
                double Hpost=0.5*entropyBern(0.95) + 0.5*entropyBern(0.05);
                double w=clamp(1.0 - Math.abs(p0-0.5)*2.0, 0.0, 1.0);
                ig += w * Math.max(0.0, H0 - Hpost);
            }
        }
        return ig;
    }

    static void applyObs(Grid belief, Grid truth, Cell pose, SensorParams sp){
        for(int dy=-sp.r; dy<=sp.r; dy++){
            for(int dx=-sp.r; dx<=sp.r; dx++){
                int x=pose.x+dx, y=pose.y+dy;
                if(!belief.inb(x,y)) continue;
                boolean occ = truth.isOcc(x,y,0.5);
                belief.set(x,y, occ ? 0.95 : 0.05);
            }
        }
    }

    static class Choice { Cell goal; List<Cell> path; double U; Choice(Cell g,List<Cell> p,double U){this.goal=g;this.path=p;this.U=U;} }

    static Choice pickGoalBudgeted(
        Grid belief, List<Cell> frontiers, Cell cur, Cell home,
        Budget bud, RobotParams rp, SensorParams sp, double lam, double occThr
    ){
        if(frontiers.isEmpty()) return null;

        // subsample
        if(frontiers.size() > 80){
            Collections.shuffle(frontiers, new Random(7));
            frontiers = frontiers.subList(0, 80);
        }

        double bestU=-1e18;
        Choice best=null;

        for(Cell g: frontiers){
            List<Cell> path = astar(belief, cur, g, occThr);
            if(path.isEmpty()) continue;
            int steps = Math.max(0, path.size()-1);
            double Ego = steps*(rp.EStep + rp.EBase);
            double Tgo = steps*rp.TStep;

            List<Cell> back = astar(belief, g, home, occThr);
            if(back.isEmpty()) continue;
            int bsteps = Math.max(0, back.size()-1);
            double Eback = bsteps*(rp.EStep + rp.EBase);
            double Tback = bsteps*rp.TStep;

            if(bud.E - (Ego+Eback) < rp.EReserve) continue;
            if(bud.T - (Tgo+Tback) < 0) continue;

            double ig = expectedIG(belief, g, sp);
            double U = ig - lam*steps;
            if(U > bestU){
                bestU=U;
                best = new Choice(g, path, U);
            }
        }
        return best;
    }

    public static void main(String[] args){
        int W=35, H=25;
        Grid belief=new Grid(W,H);
        Grid truth=new Grid(W,H);

        // truth: mostly free
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) truth.set(x,y,0.05);

        // borders
        for(int x=0;x<W;x++){ truth.set(x,0,0.95); truth.set(x,H-1,0.95); }
        for(int y=0;y<H;y++){ truth.set(0,y,0.95); truth.set(W-1,y,0.95); }

        // random rectangles
        Random rng=new Random(3);
        for(int k=0;k<8;k++){
            int x0=3 + rng.nextInt(W-10-3+1);
            int y0=3 + rng.nextInt(H-8-3+1);
            int ww=3 + rng.nextInt(7-3+1);
            int hh=2 + rng.nextInt(5-2+1);
            for(int y=y0;y<Math.min(H-1, y0+hh); y++)
                for(int x=x0;x<Math.min(W-1, x0+ww); x++)
                    truth.set(x,y,0.95);
        }

        Cell home=new Cell(2,2);
        Cell cur=home;

        RobotParams rp=new RobotParams();
        SensorParams sp=new SensorParams();
        Budget bud=new Budget(160.0, 140.0);

        applyObs(belief, truth, cur, sp);

        int stepsDone=0, goals=0;

        while(true){
            List<Cell> fr = detectFrontiers(belief, 0.35);
            if(fr.isEmpty()){
                System.out.println("No frontiers left.");
                break;
            }
            Choice ch = pickGoalBudgeted(belief, fr, cur, home, bud, rp, sp, 0.35, 0.65);
            if(ch == null){
                System.out.println("No feasible frontier under budget. Returning.");
                break;
            }
            for(int i=1;i<ch.path.size();i++){
                bud.E -= (rp.EStep + rp.EBase);
                bud.T -= rp.TStep;
                stepsDone++;
                cur = ch.path.get(i);
                applyObs(belief, truth, cur, sp);
                if(bud.E < rp.EReserve || bud.T <= 0) break;
            }
            goals++;
            if(bud.E < rp.EReserve || bud.T <= 0) break;
            if(goals >= 40){ System.out.println("Stop: demo goal limit."); break; }
        }

        List<Cell> back = astar(belief, cur, home, 0.65);
        if(!back.isEmpty()){
            for(int i=1;i<back.size();i++){
                bud.E -= (rp.EStep + rp.EBase);
                bud.T -= rp.TStep;
                cur = back.get(i);
            }
        }

        System.out.printf("Final pose: (%d,%d) steps=%d goals=%d%n", cur.x, cur.y, stepsDone, goals);
        System.out.printf("Remaining budget: E=%.2f T=%.2f%n", bud.E, bud.T);
    }
}
