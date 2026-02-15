import java.util.*;

class Fact {
    public final String name;
    public final List<String> args;
    public Fact(String name, List<String> args) {
        this.name = name;
        this.args = args;
    }
}

interface Stream {
    String getName();
    boolean precond(Map<String, String> binding, List<Fact> facts);
    Optional<List<Fact>> call(Map<String, String> binding,
                                   List<Fact> facts);
}

class MotionStream implements Stream {
    private final String name;
    public MotionStream(String name) {
        this.name = name;
    }
    @Override
    public String getName() { return name; }

    @Override
    public boolean precond(Map<String, String> binding, List<Fact> facts) {
        return true;
    }

    @Override
    public Optional<List<Fact>> call(Map<String, String> binding,
                                           List<Fact> facts) {
        String qStart = binding.get("q_start");
        String qGoal  = binding.get("q_goal");
        // TODO: Call native motion planner here.
        // Simulate success:
        List<Fact> out = new ArrayList<>();
        out.add(new Fact("reachable", Arrays.asList(qStart, qGoal)));
        return Optional.of(out);
    }
}

public class FocusedTAMP {
    public static void main(String[] args) {
        Stream motion = new MotionStream("sample-motion");
        Map<String, String> binding = new HashMap<>();
        binding.put("q_start", "q0");
        binding.put("q_goal", "q1");
        List<Fact> facts = new ArrayList<>();
        if (motion.precond(binding, facts)) {
            Optional<List<Fact>> maybeFacts = motion.call(binding, facts);
            if (maybeFacts.isPresent()) {
                for (Fact f : maybeFacts.get()) {
                    System.out.println(f.name + " " + f.args);
                }
            }
        }
    }
}
      
