
public class SystemClassifier {

    public static String classifySystem(double cT, double iEU, int rR,
                                        double th_c, double th_i, int th_r) {
        if (cT < th_c && iEU < th_i && rR == 1) return "Automation";
        if (cT >= th_c && iEU >= th_i && rR >= th_r) return "Robot";
        return "Mechatronic System";
    }

    public static void main(String[] args) {
        System.out.println("CNC line -> " +
                classifySystem(0.1, 0.05, 1, 0.3, 0.2, 2));
        System.out.println("Active suspension -> " +
                classifySystem(0.4, 0.15, 2, 0.3, 0.2, 2));
        System.out.println("Mobile robot -> " +
                classifySystem(0.7, 0.6, 5, 0.3, 0.2, 2));
    }
}
      