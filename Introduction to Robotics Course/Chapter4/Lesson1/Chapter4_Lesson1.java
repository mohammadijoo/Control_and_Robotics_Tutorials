
import java.util.*;

class Joint {
    public int parent, child, f;
    public Joint(int p, int c, int f) { parent=p; child=c; this.f=f; }
}

public class MobilityCounter {
    public static int spatialMobility(int N, List<Joint> joints) {
        int J = joints.size();
        int sumF = 0;
        for (Joint j : joints) sumF += j.f;
        return 6*(N - 1 - J) + sumF;
    }

    public static void main(String[] args) {
        int N = 7; // base + 6 moving links
        List<Joint> joints = new ArrayList<>();
        for (int i=0; i<6; i++) joints.add(new Joint(i, i+1, 1));
        System.out.println("Mobility = " + spatialMobility(N, joints));
    }
}
      