
public class SharedAutonomy1D {
    public static void main(String[] args){
        double x = 2.0, k = 1.5, alpha = 0.7;
        double T = 5.0, dt = 0.01;
        int nSteps = (int)(T/dt);

        for(int i=0; i < nSteps; i++){
            double t = i*dt;
            double uA = -k*x;
            double uH = 0.5*Math.sin(2*Math.PI*t);
            double u  = alpha*uA + (1-alpha)*uH;

            x += dt*u; // x_dot = u
        }
        System.out.println("Final state: " + x);
    }
}
      