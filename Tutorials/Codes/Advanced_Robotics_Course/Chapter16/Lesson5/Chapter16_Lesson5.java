import java.util.*;

class Transition {
    int actionId;
    List<Integer> succ;
    Transition(int actionId, List<Integer> succ) {
        this.actionId = actionId;
        this.succ = succ;
    }
}

public class CorrectByConstructionPlanner {

    public static void main(String[] args) {
        int nStates = 10;

        List<List<Transition>> delta = new ArrayList<>(nStates);
        for (int i = 0; i < nStates; ++i) {
            delta.add(new ArrayList<>());
        }

        // TODO: fill delta with transitions

        boolean[] safe = new boolean[nStates];
        boolean[] goal = new boolean[nStates];
        Arrays.fill(safe, true);
        Arrays.fill(goal, false);

        // Mark some states as unsafe / goal
        // safe[i] = false; goal[j] = true; etc.

        // W^0 = Safe ∩ Goal
        boolean[] W = new boolean[nStates];
        for (int s = 0; s < nStates; ++s) {
            W[s] = safe[s] && goal[s];
        }

        boolean changed = true;
        while (changed) {
            changed = false;
            for (int s = 0; s < nStates; ++s) {
                if (!safe[s] || W[s]) continue;
                boolean canControl = false;
                for (Transition tr : delta.get(s)) {
                    if (tr.succ.isEmpty()) continue;
                    boolean allInW = true;
                    for (int sp : tr.succ) {
                        if (!W[sp]) {
                            allInW = false;
                            break;
                        }
                    }
                    if (allInW) {
                        canControl = true;
                        break;
                    }
                }
                if (canControl) {
                    W[s] = true;
                    changed = true;
                }
            }
        }

        int[] strategy = new int[nStates];
        Arrays.fill(strategy, -1);
        for (int s = 0; s < nStates; ++s) {
            if (!W[s]) continue;
            for (Transition tr : delta.get(s)) {
                if (tr.succ.isEmpty()) continue;
                boolean allInW = true;
                for (int sp : tr.succ) {
                    if (!W[sp]) {
                        allInW = false;
                        break;
                    }
                }
                if (allInW) {
                    strategy[s] = tr.actionId;
                    break;
                }
            }
        }

        int startState = 0;
        System.out.println("Is start winning? " + W[startState]);
        System.out.println("Strategy at start: " + strategy[startState]);
    }
}
      
