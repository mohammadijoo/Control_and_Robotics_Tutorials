import java.util.*;

class State2D {
    public final int i, j;
    public State2D(int i, int j) { this.i = i; this.j = j; }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof State2D)) return false;
        State2D s = (State2D) o;
        return i == s.i && j == s.j;
    }

    @Override
    public int hashCode() { return Objects.hash(i, j); }
}

class TransitionSystem {
    Set<State2D> states = new HashSet<>();
    State2D initial;
    List<String> actions = Arrays.asList("up", "down", "left", "right");
    Map<State2D, Set<String>> label = new HashMap<>();
    Map<String, Set<State2D>> successors = new HashMap<>();

    void addState(State2D s, boolean isInitial, Set<String> props) {
        states.add(s);
        if (isInitial) initial = s;
        label.put(s, new HashSet<>(props));
    }

    void addTransition(State2D s, String act, State2D sNext) {
        String key = s.i + "," + s.j + ":" + act;
        successors.computeIfAbsent(key, k -> new HashSet<>()).add(sNext);
    }

    Set<State2D> getSuccessors(State2D s, String act) {
        String key = s.i + "," + s.j + ":" + act;
        return successors.getOrDefault(key, Collections.emptySet());
    }
}

class Buchi {
    String initial = "q0";
    Set<String> accepting = new HashSet<>(Arrays.asList("q1"));

    String delta(String q, Set<String> sigma) {
        boolean hasGoal = sigma.contains("goal");
        if (q.equals("q0")) {
            return hasGoal ? "q1" : "q0";
        } else {
            return "q1";
        }
    }
}

class ProdState {
    final State2D s;
    final String q;
    ProdState(State2D s, String q) { this.s = s; this.q = q; }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof ProdState)) return false;
        ProdState p = (ProdState) o;
        return s.equals(p.s) && q.equals(p.q);
    }

    @Override
    public int hashCode() { return Objects.hash(s, q); }
}

public class AutomataPlanner {
    static List<ProdState> bfsToAccepting(TransitionSystem ts, Buchi ba) {
        ProdState start = new ProdState(ts.initial, ba.initial);
        Queue<ProdState> q = new ArrayDeque<>();
        Map<ProdState, ProdState> pred = new HashMap<>();
        q.add(start);
        Set<ProdState> visited = new HashSet<>();
        visited.add(start);

        ProdState accepting = null;
        while (!q.isEmpty()) {
            ProdState cur = q.remove();
            if (ba.accepting.contains(cur.q)) {
                accepting = cur;
                break;
            }
            for (String act : ts.actions) {
                for (State2D sNext : ts.getSuccessors(cur.s, act)) {
                    String qNext = ba.delta(cur.q, ts.label.get(cur.s));
                    ProdState nxt = new ProdState(sNext, qNext);
                    if (!visited.contains(nxt)) {
                        visited.add(nxt);
                        pred.put(nxt, cur);
                        q.add(nxt);
                    }
                }
            }
        }

        if (accepting == null) return Collections.emptyList();

        List<ProdState> path = new ArrayList<>();
        ProdState cur = accepting;
        while (cur != null) {
            path.add(cur);
            cur = pred.get(cur);
        }
        Collections.reverse(path);
        return path;
    }

    public static void main(String[] args) {
        TransitionSystem ts = new TransitionSystem();
        // Populate ts with grid states and transitions...
        Buchi ba = new Buchi();
        List<ProdState> plan = bfsToAccepting(ts, ba);
        if (plan.isEmpty()) {
            System.out.println("No satisfying plan");
        } else {
            System.out.println("Found path length = " + plan.size());
        }
    }
}
      
