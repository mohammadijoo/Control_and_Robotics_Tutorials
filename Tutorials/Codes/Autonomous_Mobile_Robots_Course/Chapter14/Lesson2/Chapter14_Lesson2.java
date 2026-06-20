// Chapter14_Lesson2.java
// Costmaps and Inflation Concepts — grid + brushfire inflation (pure Java).
import java.util.ArrayDeque;
import java.util.Arrays;

public class Chapter14_Lesson2 {

    static int[][] makeToyMap(int w, int h) {
        int[][] g = new int[h][w];
        for (int x=0; x<w; x++) { g[0][x]=1; g[h-1][x]=1; }
        for (int y=0; y<h; y++) { g[y][0]=1; g[y][w-1]=1; }
        for (int x=10; x<50; x++) g[12][x]=1;
        for (int y=18; y<33; y++) g[y][28]=1;
        for (int x=35; x<55; x++) g[28][x]=1;
        return g;
    }

    static double[][] brushfireDistance(int[][] obs, double resolution) {
        int h = obs.length, w = obs[0].length;
        double INF = 1e18;
        double[][] dist = new double[h][w];
        for (int y=0; y<h; y++) Arrays.fill(dist[y], INF);

        ArrayDeque<int[]> q = new ArrayDeque<>();
        for (int y=0; y<h; y++) for (int x=0; x<w; x++) {
            if (obs[y][x] == 1) { dist[y][x] = 0.0; q.add(new int[]{x,y}); }
        }

        int[] dx = {-1,1,0,0,-1,1,-1,1};
        int[] dy = {0,0,-1,1,-1,-1,1,1};
        double[] step = {1,1,1,1,Math.sqrt(2),Math.sqrt(2),Math.sqrt(2),Math.sqrt(2)};

        while (!q.isEmpty()) {
            int[] c = q.removeFirst();
            int x = c[0], y = c[1];
            double dxy = dist[y][x];
            for (int k=0; k<8; k++) {
                int nx = x + dx[k], ny = y + dy[k];
                if (0 <= nx && nx < w && 0 <= ny && ny < h) {
                    double nd = dxy + step[k]*resolution;
                    if (nd < dist[ny][nx]) {
                        dist[ny][nx] = nd;
                        q.add(new int[]{nx, ny});
                    }
                }
            }
        }
        return dist;
    }

    static int inflationCost(double d, double rInscribed, double rInflation, double costScaling) {
        int lethal = 254, inscribed = 253;
        if (d <= 0.0) return lethal;
        if (d <= rInscribed) return inscribed;
        if (d > rInflation) return 0;
        double c = (inscribed - 1) * Math.exp(-costScaling * (d - rInscribed)) + 1.0;
        long ci = Math.round(c);
        if (ci < 1) ci = 1;
        if (ci > inscribed - 1) ci = inscribed - 1;
        return (int)ci;
    }

    static int[][] buildInflatedCostmap(int[][] obs, double resolution,
                                       double rInscribed, double rInflation, double costScaling) {
        double[][] dist = brushfireDistance(obs, resolution);
        int h = obs.length, w = obs[0].length;
        int[][] cm = new int[h][w];
        for (int y=0; y<h; y++) for (int x=0; x<w; x++)
            cm[y][x] = inflationCost(dist[y][x], rInscribed, rInflation, costScaling);
        return cm;
    }

    static void asciiRender(int[][] cm) {
        for (int y=0; y<cm.length; y++) {
            StringBuilder sb = new StringBuilder();
            for (int x=0; x<cm[0].length; x++) {
                int c = cm[y][x];
                char ch = ' ';
                if (c >= 254) ch = '#';
                else if (c >= 200) ch = 'X';
                else if (c >= 120) ch = '+';
                else if (c >= 40)  ch = '.';
                sb.append(ch);
            }
            System.out.println(sb.toString());
        }
    }

    public static void main(String[] args) {
        double resolution = 0.05;
        double rInscribed = 0.25;
        double rInflation = 0.60;
        double costScaling = 6.0;

        int[][] obs = makeToyMap(60, 40);
        int[][] cm = buildInflatedCostmap(obs, resolution, rInscribed, rInflation, costScaling);

        System.out.println("Inflated costmap (ASCII preview):");
        asciiRender(cm);
    }
}
