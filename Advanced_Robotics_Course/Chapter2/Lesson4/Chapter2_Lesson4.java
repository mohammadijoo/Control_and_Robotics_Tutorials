import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;

public class AStar<State> {

    public interface SuccessorFunction<State> {
        List<Transition<State>> successors(State s);
    }

    public static class Transition<State> {
        public final State next;
        public final double cost;
        public Transition(State next, double cost) {
            this.next = next;
            this.cost = cost;
        }
    }

    private static class Node<State> {
        State state;
        double g;
        double f;
        Node(State s, double g, double f) {
            this.state = s;
            this.g = g;
            this.f = f;
        }
    }

    public List<State> search(
            State start,
            Predicate<State> isGoal,
            SuccessorFunction<State> successors,
            Function<State, Double> heuristic) {

        Map<State, Double> g = new HashMap<>();
        Map<State, State> parent = new HashMap<>();
        g.put(start, 0.0);
        parent.put(start, null);

        PriorityQueue<Node<State>> open = new PriorityQueue<>(
            Comparator.comparingDouble(n -> n.f)
        );
        open.add(new Node<>(start, 0.0, heuristic.apply(start)));

        Set<State> closed = new HashSet<>();

        while (!open.isEmpty()) {
            Node<State> cur = open.poll();
            State s = cur.state;
            if (closed.contains(s)) continue;
            closed.add(s);

            if (isGoal.test(s)) {
                return reconstructPath(s, parent);
            }

            double g_s = g.get(s);
            for (Transition<State> tr : successors.successors(s)) {
                if (tr.cost < 0.0) {
                    throw new IllegalArgumentException("Negative edge costs not allowed");
                }
                double newG = g_s + tr.cost;
                Double oldG = g.get(tr.next);
                if (oldG == null || newG < oldG) {
                    g.put(tr.next, newG);
                    parent.put(tr.next, s);
                    double f = newG + heuristic.apply(tr.next);
                    open.add(new Node<>(tr.next, newG, f));
                }
            }
        }
        return null; // failure
    }

    private List<State> reconstructPath(State goal, Map<State, State> parent) {
        List<State> path = new ArrayList<>();
        State cur = goal;
        while (cur != null) {
            path.add(cur);
            cur = parent.get(cur);
        }
        Collections.reverse(path);
        return path;
    }
}
      
