import java.util.*;

public class TransitionSystemReachability {

    public static Set<Integer> forwardReach(List<List<Integer>> adj,
                                             Set<Integer> S0) {
        Set<Integer> reachable = new HashSet<>(S0);
        Queue<Integer> queue = new ArrayDeque<>(S0);

        while (!queue.isEmpty()) {
            int s = queue.poll();
            for (int sNext : adj.get(s)) {
                if (!reachable.contains(sNext)) {
                    reachable.add(sNext);
                    queue.add(sNext);
                }
            }
        }
        return reachable;
    }

    public static Set<Integer> backwardReach(List<List<Integer>> adj,
                                              Set<Integer> bad) {
        int n = adj.size();
        List<List<Integer>> radj = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            radj.add(new ArrayList<>());
        }
        for (int s = 0; s < n; ++s) {
            for (int sNext : adj.get(s)) {
                radj.get(sNext).add(s);
            }
        }

        Set<Integer> backward = new HashSet<>(bad);
        Queue<Integer> queue = new ArrayDeque<>(bad);

        while (!queue.isEmpty()) {
            int s = queue.poll();
            for (int sPrev : radj.get(s)) {
                if (!backward.contains(sPrev)) {
                    backward.add(sPrev);
                    queue.add(sPrev);
                }
            }
        }
        return backward;
    }

    public static void main(String[] args) {
        List<List<Integer>> adj = new ArrayList<>();
        adj.add(Arrays.asList(1, 2)); // 0
        adj.add(Arrays.asList(3));    // 1
        adj.add(Arrays.asList(3, 4)); // 2
        adj.add(Arrays.asList(5));    // 3
        adj.add(Arrays.asList(5, 6)); // 4
        adj.add(Arrays.asList(7));    // 5
        adj.add(Arrays.asList(7));    // 6
        adj.add(Collections.emptyList()); // 7

        Set<Integer> S0 = new HashSet<>(Collections.singletonList(0));
        Set<Integer> Bad = new HashSet<>(Collections.singletonList(6));

        Set<Integer> Rstar = forwardReach(adj, S0);
        Set<Integer> Bstar = backwardReach(adj, Bad);

        System.out.println("Reachable from S0: " + Rstar);
        System.out.println("Backward reachable to Bad: " + Bstar);

        boolean safe = true;
        for (int s0 : S0) {
            if (Bstar.contains(s0)) {
                safe = false;
            }
        }
        System.out.println("Safety holds? " + safe);
    }
}
      
