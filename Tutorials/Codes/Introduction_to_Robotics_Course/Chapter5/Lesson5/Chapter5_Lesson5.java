
import java.util.Map;

public class ArmStructure {
    private final String seq;
    private final Map<String, Double> p;
    private static final double PI = Math.PI;

    public ArmStructure(String seq, Map<String, Double> params) {
        this.seq = seq.toUpperCase();
        this.p = params;
    }

    public int dof() { return seq.length(); }

    public double workspaceVolume() {
        switch (seq) {
            case "PPP":
                return p.get("Lx") * p.get("Ly") * p.get("Lz");
            case "RPP":
                double rmin = p.get("rmin"), rmax = p.get("rmax"), Lz = p.get("Lz");
                return PI * (rmax*rmax - rmin*rmin) * Lz;
            case "RRP":
                double phimin = p.get("phimin"), phimax = p.get("phimax");
                rmin = p.get("rmin"); rmax = p.get("rmax");
                return 2*PI*(Math.cos(phimin) - Math.cos(phimax))*(Math.pow(rmax,3)-Math.pow(rmin,3))/3.0;
            case "RRP_SCARA":
                double l1 = p.get("l1"), l2 = p.get("l2");
                Lz = p.get("Lz");
                double A = PI * ((l1+l2)*(l1+l2) - Math.abs(l1-l2)*Math.abs(l1-l2));
                return A * Lz;
            default:
                return Double.NaN;
        }
    }
}
      