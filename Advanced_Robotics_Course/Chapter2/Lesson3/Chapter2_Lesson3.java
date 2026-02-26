import java.util.*;
import static java.lang.Math.*;

public class LatticePlanner {

    static final int NX = 50, NY = 30, N_THETA = 16;
    static final double DX = 1.0, DY = 1.0, L = 2.0, V = 1.0, DT = 1.0;

    static int[][] occGrid = new int[NX][NY];

    static class State {
        int ix, iy, kth;
        State(int ix, int iy, int kth) {
            this.ix = ix; this.iy = iy; this.kth = kth;
        }
        @Override public boolean equals(Object o) {
            if (!(o instanceof State)) return false;
            State s = (State)o;
            return ix == s.ix && iy == s.iy && kth == s.kth;
        }
        @Override public int hashCode() {
            return ix * 73856093 ^ iy * 19349663 ^ kth * 83492791;
        }
    }

    static class Node implements Comparable<Node> {
        State s;
        double g, f;
        Node(State s, double g, double f) {
            this.s = s; this.g = g; this.f = f;
        }
        public int compareTo(Node other) {
            return Double.compare(this.f, other.f);
        }
    }

    static class Primitive {
        double dxRef, dyRef, dthRef, length;
        Primitive(double dx, double dy, double dth, double len) {
            dxRef = dx; dyRef = dy; dthRef = dth; length = len;
        }
    }

    static boolean inBounds(int ix, int iy) {
        return ix >= 0 && ix < NX && iy >= 0 && iy < NY;
    }

    static boolean isFree(int ix, int iy) {
        return inBounds(ix, iy) && occGrid[ix][iy] == 0;
    }

    static int angleIndex(double theta) {
        double twoPi = 2.0 * PI;
        theta = (theta + twoPi) % twoPi;
        int k = (int) Math.round(theta / twoPi * N_THETA) % N_THETA;
        return k;
    }

    static double angleFromIndex(int k) {
        return 2.0 * PI * k / N_THETA;
    }

    static List<Primitive> makePrimitives() {
        double[] steer = {-0.4, 0.0, 0.4};
        List<Primitive> prims = new ArrayList<>();
        for (double phi : steer) {
            double kappa = Math.tan(phi) / L;
            if (Math.abs(kappa) < 1e-6) {
                prims.add(new Primitive(V * DT, 0.0, 0.0, V * DT));
            } else {
                double dth = V * kappa * DT;
                double thEnd = dth;
                double dx = (1.0 / kappa) * Math.sin(thEnd);
                double dy = -(1.0 / kappa) * (Math.cos(thEnd) - 1.0);
                double length = Math.abs(dth / kappa);
                prims.add(new Primitive(dx, dy, dth, length));
            }
        }
        return prims;
    }

    static double heuristic(State s, State goal) {
        double x = s.ix * DX;
        double y = s.iy * DY;
        double xg = goal.ix * DX;
        double yg = goal.iy * DY;
        return Math.hypot(x - xg, y - yg);
    }

    public static List<State> astar(State start, State goal) {
        List<Primitive> prims = makePrimitives();
        PriorityQueue<Node> open = new PriorityQueue<>();
        Map<State, Double> g = new HashMap<>();
        Map<State, State> parent = new HashMap<>();
        Set<State> closed = new HashSet<>();

        g.put(start, 0.0);
        parent.put(start, null);
        open.add(new Node(start, 0.0, heuristic(start, goal)));

        while (!open.isEmpty()) {
            Node cur = open.poll();
            if (closed.contains(cur.s)) continue;
            closed.add(cur.s);

            if (cur.s.ix == goal.ix && cur.s.iy == goal.iy) {
                List<State> path = new ArrayList<>();
                State s = cur.s;
                while (s != null) {
                    path.add(s);
                    s = parent.get(s);
                }
                Collections.reverse(path);
                return path;
            }

            double theta = angleFromIndex(cur.s.kth);
            for (Primitive p : prims) {
                double gx = cur.s.ix * DX;
                double gy = cur.s.iy * DY;
                double dx = Math.cos(theta) * p.dxRef - Math.sin(theta) * p.dyRef;
                double dy = Math.sin(theta) * p.dxRef + Math.cos(theta) * p.dyRef;
                double gx2 = gx + dx;
                double gy2 = gy + dy;
                double th2 = theta + p.dthRef;

                int ix2 = (int) Math.round(gx2 / DX);
                int iy2 = (int) Math.round(gy2 / DY);
                int kth2 = angleIndex(th2);
                if (!isFree(ix2, iy2)) continue;

                State s2 = new State(ix2, iy2, kth2);
                double newG = g.get(cur.s) + p.length;
                if (!g.containsKey(s2) || newG < g.get(s2)) {
                    g.put(s2, newG);
                    parent.put(s2, cur.s);
                    double f = newG + heuristic(s2, goal);
                    open.add(new Node(s2, newG, f));
                }
            }
        }
        return null;
    }

    public static void main(String[] args) {
        State start = new State(2, 2, angleIndex(0.0));
        State goal = new State(40, 20, angleIndex(0.0));
        List<State> path = astar(start, goal);
        if (path != null) {
            System.out.println("Found path of length " + path.size());
        } else {
            System.out.println("No path found.");
        }
    }
}
      
