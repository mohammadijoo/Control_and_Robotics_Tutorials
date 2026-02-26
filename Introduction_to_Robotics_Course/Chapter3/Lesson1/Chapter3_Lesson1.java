abstract class Robot {
    String name;
    int dofBase, dofJoints, nAgents;
    boolean humanoid;

    Robot(String name, int dofBase, int dofJoints, int nAgents, boolean humanoid) {
        this.name = name; this.dofBase = dofBase; this.dofJoints = dofJoints;
        this.nAgents = nAgents; this.humanoid = humanoid;
    }

    int dofTotal() { return dofBase + dofJoints; }

    String family() {
        if (nAgents > 1) return "Swarm";
        if (humanoid) return "Humanoid";
        if (dofBase > 0 && dofJoints == 0) return "Mobile robot";
        if (dofBase == 0 && dofJoints > 0) return "Manipulator";
        if (dofBase > 0 && dofJoints > 0) return "Mobile manipulator";
        return "Other";
    }
}

public class TaxonomyDemo {
    public static void main(String[] args) {
        Robot arm = new Robot("6R arm", 0, 6, 1, false){};
        Robot mobile = new Robot("Wheeled base", 3, 0, 1, false){};
        Robot humanoid = new Robot("Humanoid", 6, 22, 1, true){};
        Robot swarm = new Robot("Swarm", 3, 0, 50, false){};

        Robot[] robots = {arm, mobile, humanoid, swarm};
        for (Robot r : robots) {
            System.out.println(r.name + " => " + r.family() + " (DoF " + r.dofTotal() + ")");
        }
    }
}
      