public class LifeCycleCost {

    public static double discountedSum(double[] yearlyCosts, double rho) {
        double lcc = 0.0;
        for (int k = 0; k < yearlyCosts.length; ++k) {
            int year = k + 1;
            lcc += yearlyCosts[k] / Math.pow(1.0 + rho, year);
        }
        return lcc;
    }

    public static void main(String[] args) {
        double C_dev = 50000.0;
        double C_acq = 80000.0;
        double rho = 0.05;

        // Example expected yearly operating + maintenance + downtime costs
        double[] yearly = new double[] {
            15000.0, 15000.0, 16000.0, 17000.0, 18000.0
        };

        double lcc_oper = discountedSum(yearly, rho);
        double lcc_total = C_dev + C_acq + lcc_oper;

        System.out.println("Discounted operating-related LCC: " + lcc_oper);
        System.out.println("Total LCC (approx): " + lcc_total);
    }
}
      
