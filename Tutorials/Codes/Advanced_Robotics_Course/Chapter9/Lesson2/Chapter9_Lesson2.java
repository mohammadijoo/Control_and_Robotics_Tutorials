import java.util.*;

enum Mode {
    TRANSIT,
    TRANSFER
}

final class HybridState {
    final Mode mode;
    final int rx, ry;
    final int ox, oy;

    HybridState(Mode mode, int rx, int ry, int ox, int oy) {
        this.mode = mode;
        this.rx = rx;
        this.ry = ry;
        this.ox = ox;
        this.oy = oy;
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof HybridState)) return false;
        HybridState s = (HybridState) o;
        return mode == s.mode &&
               rx == s.rx && ry == s.ry &&
               ox == s.ox && oy == s.oy;
    }

    @Override
    public int hashCode() {
        return Objects.hash(mode, rx, ry, ox, oy);
    }

    @Override
    public String toString() {
        return String.format("HybridState(%s, r=(%d,%d), o=(%d,%d))",
                             mode, rx, ry, ox, oy);
    }
}

public class HybridGridPlanner {
    static final int W = 7, H = 7;
    static final int FREE = 0, OBST = 1;
    static int[][] grid = new int[H][W];

    static boolean inBounds(int x, int y) {
        return 0 <= x && x < W && 0 <= y && y < H;
    }

    static boolean isFree(int x, int y) {
        return inBounds(x, y) && grid[y][x] == FREE;
    }

    static List<AbstractMap.SimpleEntry<String, HybridState>>
    neighbors(HybridState s, int gx, int gy) {
        List<AbstractMap.SimpleEntry<String, HybridState>> succ =
            new ArrayList<>();

        int[][] dirs = { {1,0},{-1,0},{0,1},{0,-1} };
        for (int[] d : dirs) {
            int nx = s.rx + d[0];
            int ny = s.ry + d[1];
            if (!isFree(nx, ny)) continue;

            HybridState ns;
            if (s.mode == Mode.TRANSIT) {
                ns = new HybridState(Mode.TRANSIT, nx, ny, s.ox, s.oy);
            } else {
                ns = new HybridState(Mode.TRANSFER, nx, ny, nx, ny);
            }
            succ.add(new AbstractMap.SimpleEntry<>("move", ns));
        }

        if (s.mode == Mode.TRANSIT && s.rx == s.ox && s.ry == s.oy) {
            HybridState ns = new HybridState(Mode.TRANSFER, s.rx, s.ry, s.rx, s.ry);
            succ.add(new AbstractMap.SimpleEntry<>("pick", ns));
        }

        if (s.mode == Mode.TRANSFER && s.rx == gx && s.ry == gy) {
            HybridState ns = new HybridState(Mode.TRANSIT, s.rx, s.ry, s.rx, s.ry);
            succ.add(new AbstractMap.SimpleEntry<>("place", ns));
        }

        return succ;
    }

    public static void main(String[] args) {
        // Initialize grid
        for (int y = 0; y < H; ++y)
            Arrays.fill(grid[y], FREE);
        grid[3][3] = OBST;

        int sx = 0, sy = 0;
        int ox = 2, oy = 2;
        int gx = 6, gy = 6;

        HybridState start = new HybridState(Mode.TRANSIT, sx, sy, ox, oy);

        Queue<HybridState> frontier = new ArrayDeque<>();
        frontier.add(start);

        Map<HybridState, AbstractMap.SimpleEntry<HybridState, String>> parent =
            new HashMap<>();
        parent.put(start, new AbstractMap.SimpleEntry<>(start, "start"));

        HybridState goal = null;

        while (!frontier.isEmpty()) {
            HybridState s = frontier.remove();
            if (s.mode == Mode.TRANSIT && s.ox == gx && s.oy == gy) {
                goal = s;
                break;
            }

            for (AbstractMap.SimpleEntry<String, HybridState> e :
                    neighbors(s, gx, gy)) {
                String act = e.getKey();
                HybridState ns = e.getValue();
                if (!parent.containsKey(ns)) {
                    parent.put(ns, new AbstractMap.SimpleEntry<>(s, act));
                    frontier.add(ns);
                }
            }
        }

        if (goal == null) {
            System.out.println("No hybrid plan found.");
            return;
        }

        List<AbstractMap.SimpleEntry<String, HybridState>> plan = new ArrayList<>();
        HybridState cur = goal;
        while (!cur.equals(start)) {
            AbstractMap.SimpleEntry<HybridState, String> par = parent.get(cur);
            plan.add(new AbstractMap.SimpleEntry<>(par.getValue(), cur));
            cur = par.getKey();
        }
        Collections.reverse(plan);

        System.out.println("Plan length: " + plan.size());
        for (AbstractMap.SimpleEntry<String, HybridState> step : plan) {
            System.out.println(step.getKey() + " : " + step.getValue());
        }
    }
}
      
