import org.ejml.simple.SimpleMatrix;
import java.util.List;

interface DynamicsFunctor {
    SimpleMatrix forwardDynamics(SimpleMatrix q,
                                 SimpleMatrix dq,
                                 SimpleMatrix tau);
}

class Link {
    public String name;
    public int index;
    public Link(String name, int index) {
        this.name = name;
        this.index = index;
    }
}

class Joint {
    public String name;
    public int parentIndex;
    public int childIndex;
    public int dof;
    public Joint(String name, int parentIndex, int childIndex, int dof) {
        this.name = name;
        this.parentIndex = parentIndex;
        this.childIndex = childIndex;
        this.dof = dof;
    }
}

class RobotModelJava {
    private List<Link> links;
    private List<Joint> joints;
    private DynamicsFunctor dynamics;

    public RobotModelJava(List<Link> links,
                          List<Joint> joints,
                          DynamicsFunctor dynamics) {
        this.links = links;
        this.joints = joints;
        this.dynamics = dynamics;
    }

    public SimpleMatrix forwardDynamics(SimpleMatrix q,
                                        SimpleMatrix dq,
                                        SimpleMatrix tau) {
        return dynamics.forwardDynamics(q, dq, tau);
    }
}
      
