public enum JointType {
    REVOLUTE, PRISMATIC, FIXED
}

public class Joint {
    public JointType type;
    public double offset;

    public Joint(JointType type, double offset) {
        this.type = type;
        this.offset = offset;
    }
}

public class Link {
    public String name;
    public double length;
    public Joint joint;

    public Link(String name, double length, Joint joint) {
        this.name = name;
        this.length = length;
        this.joint = joint;
    }
}

public class SerialChain {
    private List<Link> links;

    public SerialChain(List<Link> links) {
        this.links = links;
    }

    // Simple 4x4 transform represented as double[4][4]
    private static double[][] identity4() {
        double[][] I = new double[4][4];
        for (int i = 0; i < 4; ++i) {
            I[i][i] = 1.0;
        }
        return I;
    }

    private static double[][] rotZ(double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        double[][] T = identity4();
        T[0][0] = c;  T[0][1] = -s;
        T[1][0] = s;  T[1][1] =  c;
        return T;
    }

    private static double[][] transX(double d) {
        double[][] T = identity4();
        T[0][3] = d;
        return T;
    }

    private static double[][] matMul(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 4; ++k) {
                    sum += A[i][k] * B[k][j];
                }
                C[i][j] = sum;
            }
        }
        return C;
    }

    public List<double[][]> forwardKinematics(double[] q) {
        if (q.length != links.size()) {
            throw new IllegalArgumentException("Dimension mismatch");
        }
        List<double[][]> Ts = new ArrayList<>();
        double[][] T = identity4(); // base
        Ts.add(T);

        for (int i = 0; i < links.size(); ++i) {
            Link L = links.get(i);
            double qi = q[i];
            double[][] A = identity4();

            if (L.joint.type == JointType.REVOLUTE) {
                double theta = L.joint.offset + qi;
                A = matMul(rotZ(theta), transX(L.length));
            } else if (L.joint.type == JointType.PRISMATIC) {
                double d = L.joint.offset + qi;
                A = transX(d);
            }
            T = matMul(T, A);
            Ts.add(T);
        }
        return Ts;
    }
}
      
