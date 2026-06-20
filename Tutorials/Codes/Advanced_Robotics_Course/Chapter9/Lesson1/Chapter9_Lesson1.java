import java.util.*;

class Action {
    public final String name;
    public final Set<String> pre;
    public final Set<String> add;
    public final Set<String> del;

    public Action(String name, Set<String> pre, Set<String> add, Set<String> del) {
        this.name = name;
        this.pre = pre;
        this.add = add;
        this.del = del;
    }

    public boolean applicable(Set<String> s) {
        return s.containsAll(pre);
    }

    public Set<String> apply(Set<String> s) {
        Set<String> out = new HashSet<>(s);
        out.removeAll(del);
        out.addAll(add);
        return out;
    }
}

class Node {
    public final Set<String> state;
    public final List<Action> plan;

    public Node(Set<String> state, List<Action> plan) {
        this.state = state;
        this.plan = plan;
    }
}

public class TAMPPlanner {

    // Stub for geometric feasibility: in practice this would call a C++ node
    // via ROS services or gRPC to check motion-planning feasibility.
    public static boolean geomFeasible(Set<String> s, Action a, Set<String> sNext) {
        // TODO: connect to geometric planner; here we just return true
        return true;
    }

    public static Node bfs(Set<String> init, Set<String> goal, List<Action> actions) {
        Queue<Node> q = new ArrayDeque<>();
        q.add(new Node(init, new ArrayList<>()));
        Set<Set<String>> visited = new HashSet<>();
        visited.add(init);
        while (!q.isEmpty()) {
            Node n = q.remove();
            if (n.state.containsAll(goal)) {
                return n;
            }
            for (Action a : actions) {
                if (!a.applicable(n.state)) continue;
                Set<String> sNext = a.apply(n.state);
                if (visited.contains(sNext)) continue;
                if (!geomFeasible(n.state, a, sNext)) continue;
                visited.add(sNext);
                List<Action> newPlan = new ArrayList<>(n.plan);
                newPlan.add(a);
                q.add(new Node(sNext, newPlan));
            }
        }
        return null;
    }

    public static void main(String[] args) {
        Set<String> init = new HashSet<>(Arrays.asList("HandEmpty", "On(Box,Table)"));
        Set<String> goal = new HashSet<>(Arrays.asList("On(Box,Shelf)"));
        List<Action> actions = new ArrayList<>();
        actions.add(new Action(
                "PickFromTable",
                new HashSet<>(Arrays.asList("HandEmpty", "On(Box,Table)")),
                new HashSet<>(Arrays.asList("Holding(Box)")),
                new HashSet<>(Arrays.asList("HandEmpty", "On(Box,Table)"))
        ));
        actions.add(new Action(
                "PlaceOnShelf",
                new HashSet<>(Arrays.asList("Holding(Box)")),
                new HashSet<>(Arrays.asList("On(Box,Shelf)", "HandEmpty")),
                new HashSet<>(Arrays.asList("Holding(Box)"))
        ));

        Node sol = bfs(init, goal, actions);
        if (sol == null) {
            System.out.println("No plan found.");
        } else {
            System.out.println("Plan:");
            for (Action a : sol.plan) {
                System.out.println(" - " + a.name);
            }
        }
    }
}
      
