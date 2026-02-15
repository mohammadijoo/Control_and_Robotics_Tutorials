import org.ejml.simple.SimpleMatrix;

public class FloatingBaseMomentum {
    public static void main(String[] args) {
        int nJoints = 12;
        int ndof = 6 + nJoints;

        // Generalized coordinates q and velocities v
        SimpleMatrix q = new SimpleMatrix(ndof, 1);
        SimpleMatrix v = new SimpleMatrix(ndof, 1);

        // Example inertia matrix H(q); identity for demonstration
        SimpleMatrix H = SimpleMatrix.identity(ndof);

        // Set example base twist (omega_x, omega_y, omega_z, v_x, v_y, v_z)
        v.set(0, 0.0);
        v.set(1, 0.0);
        v.set(2, 0.5);
        v.set(3, 0.1);
        v.set(4, 0.0);
        v.set(5, 0.0);

        // Set joint velocities
        for (int i = 6; i < ndof; ++i) {
            v.set(i, 0.2);
        }

        // Generalized momentum p = H(q) v
        SimpleMatrix p = H.mult(v);

        System.out.println("v = ");
        v.print();
        System.out.println("p = ");
        p.print();
    }
}
      
