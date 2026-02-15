
public class MinimalRobot {
    private double x_r;

    public MinimalRobot(double x0){
        this.x_r = x0;
    }

    public double sense(double x_e){
        return x_e - x_r;
    }

    public double policy(double y, double alpha, double r){
        double phi = 0.5 * y;
        return alpha * r + (1.0 - alpha) * phi;
    }

    public void actuate(double u, double dt){
        x_r += u * dt;
    }

    public static void main(String[] args){
        MinimalRobot robot = new MinimalRobot(0.0);
        double x_e = 10.0;

        for(int k=0; k<50; k++){
            double y = robot.sense(x_e);
            double u = robot.policy(y, 0.2, 0.0);
            robot.actuate(u, 0.1);
        }
        System.out.println("Final robot position: " + robot.x_r);
    }
}
      