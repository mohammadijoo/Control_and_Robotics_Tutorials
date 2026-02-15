public class Planar2RWorkspace {

    public static void main(String[] args) {
        double l1 = 1.0;
        double l2 = 0.7;

        double th1Min = -Math.PI;
        double th1Max =  Math.PI;
        double th2Min = -Math.PI;
        double th2Max =  Math.PI;

        int n1 = 300;
        int n2 = 300;

        StringBuilder sb = new StringBuilder();
        sb.append("x,y,manip\n");

        for (int i = 0; i < n1; i++) {
            double th1 = th1Min + (th1Max - th1Min) * i / (n1 - 1);
            for (int j = 0; j < n2; j++) {
                double th2 = th2Min + (th2Max - th2Min) * j / (n2 - 1);

                double x = l1 * Math.cos(th1) + l2 * Math.cos(th1 + th2);
                double y = l1 * Math.sin(th1) + l2 * Math.sin(th1 + th2);

                double j11 = -l1 * Math.sin(th1) - l2 * Math.sin(th1 + th2);
                double j12 = -l2 * Math.sin(th1 + th2);
                double j21 =  l1 * Math.cos(th1) + l2 * Math.cos(th1 + th2);
                double j22 =  l2 * Math.cos(th1 + th2);

                double detJ = j11 * j22 - j12 * j21;
                double manip = Math.abs(detJ);

                sb.append(x).append(",").append(y).append(",").append(manip).append("\n");
            }
        }

        // Save to file (omitting try-with-resources boilerplate for brevity)
        try {
            java.nio.file.Files.write(
                java.nio.file.Paths.get("workspace_2R_java.csv"),
                sb.toString().getBytes()
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
      
