public class CostModel {

    public static double unitCost(double F, double c_v, double c_o,
                                  double Y, double N) {
        return F / N + c_v / Y + c_o;
    }

    public static void main(String[] args) {
        double F_A = 50000.0, cA = 250.0, Y_A = 0.95, c_oA = 40.0;
        double F_B = 15000.0, cB = 280.0, Y_B = 0.98, c_oB = 35.0;

        for (int N = 500; N <= 3000; N += 500) {
            double cA_N = unitCost(F_A, cA, c_oA, Y_A, N);
            double cB_N = unitCost(F_B, cB, c_oB, Y_B, N);
            System.out.println("N=" + N
                    + "  c_A=" + cA_N
                    + "  c_B=" + cB_N);
        }
    }
}
      
