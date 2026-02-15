
public class RuntimeEstimate {
    public static double runtimeHours(double voltage, double capacityAh, double avgPowerW){
        double Eb_Wh = voltage * capacityAh; // Wh
        return Eb_Wh / avgPowerW;
    }
    public static void main(String[] args){
        double V = 24.0;      // battery voltage
        double C = 18.0;      // Ah
        double Pavg = 120.0;  // W
        System.out.println("Estimated runtime (h): " + runtimeHours(V, C, Pavg));
    }
}
      