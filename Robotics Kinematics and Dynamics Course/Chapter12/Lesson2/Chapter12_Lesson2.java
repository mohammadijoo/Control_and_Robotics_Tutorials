import org.ejml.simple.SimpleMatrix;

class LinkDynamics {
    SimpleMatrix F;        // 3x1
    SimpleMatrix N;        // 3x1
    SimpleMatrix R_ip1_i;  // 3x3
    SimpleMatrix p_i;      // 3x1
    SimpleMatrix z;        // 3x1
    char jointType;        // 'R' or 'P'
}

public class NewtonEulerBackward {

    static class Result {
        SimpleMatrix[] fList;
        SimpleMatrix[] nList;
        double[] tauList;
    }

    public static Result compute(LinkDynamics[] links,
                                 SimpleMatrix fTip,
                                 SimpleMatrix nTip) {
        int n = links.length;
        SimpleMatrix[] fList = new SimpleMatrix[n];
        SimpleMatrix[] nList = new SimpleMatrix[n];
        double[] tauList = new double[n];

        SimpleMatrix fNext = fTip.copy();
        SimpleMatrix nNext = nTip.copy();

        for (int i = n - 1; i >= 0; --i) {
            LinkDynamics L = links[i];

            SimpleMatrix fChild_i = L.R_ip1_i.mult(fNext);
            SimpleMatrix nChild_i = L.R_ip1_i.mult(nNext)
                    .plus(L.p_i.cross(fChild_i)); // cross: helper returning 3x1

            SimpleMatrix f_i = L.F.plus(fChild_i);
            SimpleMatrix n_i = L.N.plus(nChild_i);

            fList[i] = f_i;
            nList[i] = n_i;

            double tau_i;
            if (L.jointType == 'R') {
                tau_i = L.z.dot(n_i);
            } else {
                tau_i = L.z.dot(f_i);
            }
            tauList[i] = tau_i;

            fNext = f_i;
            nNext = n_i;
        }

        Result res = new Result();
        res.fList = fList;
        res.nList = nList;
        res.tauList = tauList;
        return res;
    }
}
      
