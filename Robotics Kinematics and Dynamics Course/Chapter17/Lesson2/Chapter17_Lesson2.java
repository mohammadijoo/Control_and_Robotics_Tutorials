import org.ejml.simple.SimpleMatrix;
import java.util.ArrayList;
import java.util.List;

class Link {
    int parent;
    SimpleMatrix S;              // 6x1
    SimpleMatrix T_parent_to_i0; // 4x4

    Link(int parent, SimpleMatrix S, SimpleMatrix T_parent_to_i0) {
        this.parent = parent;
        this.S = S;
        this.T_parent_to_i0 = T_parent_to_i0;
    }
}

class KinematicTree {
    List<Link> links;

    KinematicTree(List<Link> links) {
        this.links = links;
    }

    void forwardKinematics(SimpleMatrix T_base,
                           SimpleMatrix v_base,
                           double[] q,
                           List<SimpleMatrix> T_world,
                           List<SimpleMatrix> v_world) {
        int n = links.size();
        T_world.clear();
        v_world.clear();
        for (int i = 0; i < n; ++i) {
            T_world.add(SimpleMatrix.identity(4));
            v_world.add(new SimpleMatrix(6, 1));
        }

        T_world.set(0, T_base);
        v_world.set(0, v_base);

        for (int i = 1; i < n; ++i) {
            Link link = links.get(i);
            int parent = link.parent;

            SimpleMatrix T_joint = expSE3(link.S, q[i - 1]); // user-defined
            SimpleMatrix T_pi = T_world.get(parent)
                    .mult(link.T_parent_to_i0)
                    .mult(T_joint);
            T_world.set(i, T_pi);

            SimpleMatrix X_i_parent = adjoint(link.T_parent_to_i0.mult(T_joint));
            SimpleMatrix v_i = X_i_parent.mult(v_world.get(parent));
            v_world.set(i, v_i);
        }
    }
}
      
