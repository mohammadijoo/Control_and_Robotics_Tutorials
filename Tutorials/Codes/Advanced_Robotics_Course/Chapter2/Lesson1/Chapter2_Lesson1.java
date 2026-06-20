import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;

public class DiscretePlanner {
    public static void main(String[] args) {
        SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> g =
            new SimpleDirectedWeightedGraph<>(DefaultWeightedEdge.class);

        // States
        String s0 = "(0,0)";
        String s1 = "(1,0)";
        String s2 = "(1,1)";
        String s3 = "(2,1)";

        g.addVertex(s0);
        g.addVertex(s1);
        g.addVertex(s2);
        g.addVertex(s3);

        // Transitions
        DefaultWeightedEdge e01 = g.addEdge(s0, s1);
        g.setEdgeWeight(e01, 1.0);

        DefaultWeightedEdge e12 = g.addEdge(s1, s2);
        g.setEdgeWeight(e12, 1.0);

        DefaultWeightedEdge e02 = g.addEdge(s0, s2);
        g.setEdgeWeight(e02, 2.5);

        DefaultWeightedEdge e23 = g.addEdge(s2, s3);
        g.setEdgeWeight(e23, 1.0);

        // Dijkstra shortest path
        DijkstraShortestPath<String, DefaultWeightedEdge> dsp =
            new DijkstraShortestPath<>(g);

        GraphPath<String, DefaultWeightedEdge> path =
            dsp.getPath(s0, s3);

        System.out.println("Cost: " + path.getWeight());
        System.out.println("Path: " + path.getVertexList());
    }
}
      
