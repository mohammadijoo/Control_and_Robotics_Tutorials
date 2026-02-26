
import java.util.*;

class Joint {
    String name;
    int fi;
    Joint(String name, int fi) { this.name = name; this.fi = fi; }
}

public class MobilityGK {
    static int mobilitySpatial(int L, List<Joint> joints) {
        int J = joints.size();
        int sumF = 0;
        for (Joint j : joints) sumF += j.fi;
        return 6 * (L - 1 - J) + sumF;
    }
    public static void main(String[] args) {
        List<Joint> serial = new ArrayList<>();
        for (int i=0;i<6;i++) serial.add(new Joint("R"+(i+1), 1));
        System.out.println("Serial M = " + mobilitySpatial(7, serial));
    }
}
      