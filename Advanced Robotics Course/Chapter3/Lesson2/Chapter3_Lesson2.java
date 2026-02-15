import java.util.*;
import java.util.stream.Collectors;

class Point2D {
    double x, y;
    Point2D(double x, double y) { this.x = x; this.y = y; }
}

class Box {
    double xmin, ymin, xmax, ymax;
    Box(double xmin, double ymin, double xmax, double ymax) {
        this.xmin = xmin; this.ymin = ymin;
        this.xmax = xmax; this.ymax = ymax;
    }
}

class Edge {
    int v;
    double cost;
    Edge(int v, double cost) { this.v = v; this.cost = cost; }
}

public class PRMPlanner {

    static boolean inCollision(Point2D q, List<Box> obstacles) {
        for (Box b : obstacles) {
            if (b.xmin <= q.x && q.x <= b.xmax
                    && b.ymin <= q.y && q.y <= b.ymax) {
                return true;
            }
        }
        return false;
    }

    static double euclidean(Point2D a, Point2D b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    static List<Point2D> interpolate(Point2D a, Point2D b, int nSteps) {
        List<Point2D> pts = new ArrayList<>(nSteps + 1);
        for (int i = 0; i <= nSteps; ++i) {
            double t = (double) i / (double) nSteps;
            double x = (1.0 - t) * a.x + t * b.x;
            double y = (1.0 - t) * a.y + t * b.y;
            pts.add(new Point2D(x, y));
        }
        return pts;
    }

    static boolean edgeCollisionFree(Point2D a,
                                     Point2D b,
                                     List<Box> obstacles,
                                     int nSteps) {
        for (Point2D q : interpolate(a, b, nSteps)) {
            if (inCollision(q, obstacles)) {
                return false;
            }
        }
        return true;
    }

    static List<List<Edge> > buildPRM(List<Point2D> samples,
                                           double r,
                                           List<Box> obstacles) {
        int n = samples.size();
        List<List<Edge> > graph = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            graph.add(new ArrayList<>());
        }
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                double d = euclidean(samples.get(i), samples.get(j));
                if (d <= r) {
                    if (edgeCollisionFree(samples.get(i), samples.get(j),
                                          obstacles, 20)) {
                        graph.get(i).add(new Edge(j, d));
                        graph.get(j).add(new Edge(i, d));
                    }
                }
            }
        }
        return graph;
    }

    static List<Integer> dijkstra(List<List<Edge> > graph,
                                   int start,
                                   int goal) {
        int n = graph.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(prev, -1);
        dist[start] = 0.0;

        PriorityQueue<int[]> pq = new PriorityQueue<>(
            Comparator.comparingDouble(a -> dist[a[0]])
        );
        pq.add(new int[]{start});

        while (!pq.isEmpty()) {
            int u = pq.poll()[0];
            if (u == goal) break;
            for (Edge e : graph.get(u)) {
                int v = e.v;
                double nd = dist[u] + e.cost;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    prev[v] = u;
                    pq.add(new int[]{v});
                }
            }
        }

        if (!Double.isFinite(dist[goal])) {
            return Collections.emptyList();
        }
        List<Integer> path = new ArrayList<>();
        for (int u = goal; u != -1; u = prev[u]) {
            path.add(u);
        }
        Collections.reverse(path);
        return path;
    }

    public static void main(String[] args) {
        // Example usage
        List<Box> obstacles = List.of(new Box(0.3, 0.3, 0.6, 0.7));
        List<Point2D> samples = new ArrayList<>();
        Random rng = new Random(0);
        while (samples.size() < 200) {
            double x = rng.nextDouble();
            double y = rng.nextDouble();
            Point2D q = new Point2D(x, y);
            if (!inCollision(q, obstacles)) {
                samples.add(q);
            }
        }
        double r = 0.15;
        var graph = buildPRM(samples, r, obstacles);
        // Add start and goal, run Dijkstra, etc.
    }
}
      
