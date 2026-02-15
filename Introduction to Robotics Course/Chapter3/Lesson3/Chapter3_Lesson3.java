public class HighLevelDeformationDemo {

    static double cantileverTipDeflection(double F, double L, double E, double I){
        return F * Math.pow(L,3) / (3.0 * E * I);
    }

    static double[] continuumTipPosition(double kappa, double phi, double L){
        if (Math.abs(kappa) < 1e-9){
            return new double[]{0.0, 0.0, L};
        }
        double x = (1.0/kappa) * (1 - Math.cos(kappa*L)) * Math.cos(phi);
        double y = (1.0/kappa) * (1 - Math.cos(kappa*L)) * Math.sin(phi);
        double z = (1.0/kappa) * Math.sin(kappa*L);
        return new double[]{x, y, z};
    }

    public static void main(String[] args){
        double F=10.0, L=0.5, E=70e9, I=2e-10;
        System.out.println("Deflection: " + cantileverTipDeflection(F,L,E,I));

        double kappa=4.0, phi=Math.PI/6;
        double[] p = continuumTipPosition(kappa,phi,L);
        System.out.printf("Continuum tip: [%.5f, %.5f, %.5f]%n", p[0], p[1], p[2]);
    }
}
      