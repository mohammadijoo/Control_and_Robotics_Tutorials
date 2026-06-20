interface Formula { }

final class AP implements Formula {
    public final String name;
    public AP(String name) { this.name = name; }
}

final class Not implements Formula {
    public final Formula sub;
    public Not(Formula sub) { this.sub = sub; }
}

final class And implements Formula {
    public final Formula left, right;
    public And(Formula left, Formula right) {
        this.left = left; this.right = right;
    }
}

// Extend with Next, Globally, Eventually, Until, etc.

import java.util.List;
import java.util.Set;

class LTL {
    public static boolean holds(Formula f,
                                List<Set<String>> trace,
                                int i) {
        if (f instanceof AP) {
            return trace.get(i).contains(((AP) f).name);
        } else if (f instanceof Not) {
            return !holds(((Not) f).sub, trace, i);
        } else if (f instanceof And) {
            And g = (And) f;
            return holds(g.left, trace, i) && holds(g.right, trace, i);
        }
        // Add remaining cases...
        throw new IllegalArgumentException("Unknown formula");
    }
}
      
