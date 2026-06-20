import java.util.ArrayList;
import java.util.List;

// Minimal linear algebra can be provided by a library such as EJML.
// Here we focus on the combinatorial Gruebler/Kutzbach formula.

class Joint {
    int dof;
    String name;

    Joint(int dof, String name) {
        this.dof = dof;
        this.name = name;
    }
}

class Mechanism {
    private int nLinks;
    private int lambdaDim;
    private List<Joint> joints = new ArrayList<>();

    Mechanism(int nLinks, int lambdaDim) {
        this.nLinks = nLinks;
        this.lambdaDim = lambdaDim;
    }

    void addJoint(Joint j) {
        joints.add(j);
    }

    int mobilityGruebler() {
        int J = joints.size();
        int fSum = 0;
        for (Joint j : joints) {
            fSum += j.dof;
        }
        return lambdaDim * (nLinks - 1 - J) + fSum;
    }
}

public class MobilityDemo {
    public static void main(String[] args) {
        Mechanism planarArm = new Mechanism(3, 3); // planar 2R arm
        planarArm.addJoint(new Joint(1, "R1"));
        planarArm.addJoint(new Joint(1, "R2"));
        System.out.println("Planar 2R mobility: " + planarArm.mobilityGruebler());
    }
}
      
