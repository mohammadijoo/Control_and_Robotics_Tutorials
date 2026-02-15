import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;

public class SerialChain {
    public static class Transform {
        // Minimal 4x4 matrix representation
        public double[][] data = new double[4][4];
        public Transform() {
            for (int i = 0; i < 4; ++i) data[i][i] = 1.0;
        }
        public static Transform multiply(Transform A, Transform B) {
            Transform C = new Transform();
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    C.data[i][j] = 0.0;
                    for (int k = 0; k < 4; ++k) {
                        C.data[i][j] += A.data[i][k] * B.data[k][j];
                    }
                }
            }
            return C;
        }
    }

    private int nLinks;
    private int[] parent;
    private List<DoubleFunction<Transform>> jointTF;

    public SerialChain(int nLinks) {
        this.nLinks = nLinks;
        this.parent = new int[nLinks];
        this.jointTF = new ArrayList<>(nLinks);
        parent[0] = -1;
        for (int i = 1; i < nLinks; ++i) parent[i] = i - 1;
        for (int i = 0; i < nLinks; ++i) jointTF.add(null);
    }

    public void setJointTransform(int i, DoubleFunction<Transform> f) {
        if (i <= 0 || i >= nLinks) {
            throw new IllegalArgumentException("Invalid link index");
        }
        jointTF.set(i, f);
    }

    public Transform[] forwardTransforms(double[] q) {
        Transform[] T = new Transform[nLinks];
        for (int i = 0; i < nLinks; ++i) T[i] = new Transform();
        for (int i = 1; i < nLinks; ++i) {
            int p = parent[i];
            Transform Tpi = jointTF.get(i).apply(q[i - 1]);
            T[i] = Transform.multiply(T[p], Tpi);
        }
        return T;
    }

    public static Transform planarR(double theta, double a) {
        Transform T = new Transform();
        double c = Math.cos(theta), s = Math.sin(theta);
        T.data[0][0] = c;  T.data[0][1] = -s;
        T.data[1][0] = s;  T.data[1][1] =  c;
        T.data[0][3] = a;
        return T;
    }

    public static void main(String[] args) {
        SerialChain chain = new SerialChain(4);
        double[] a = {1.0, 1.0, 1.0};
        for (int i = 1; i <= 3; ++i) {
            final double ai = a[i - 1];
            chain.setJointTransform(i, (double q_i) -> planarR(q_i, ai));
        }
        double[] q = {0.2, 0.4, -0.3};
        Transform[] T = chain.forwardTransforms(q);
        Transform Tee = T[3];
        System.out.println("End-effector x = " + Tee.data[0][3]
                           + ", y = " + Tee.data[1][3]);
    }
}
      
