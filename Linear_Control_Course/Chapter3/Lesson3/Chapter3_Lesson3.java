public class DCMotorModel {
    private double J = 0.01;
    private double b = 0.1;
    private double R_a = 1.0;
    private double L_a = 0.5;
    private double K_t = 0.01;
    private double K_e = 0.01;

    private double theta = 0.0;
    private double omega = 0.0;
    private double i_a = 0.0;

    public void step(double v_a, double T_load, double dt) {
        double dtheta = omega;
        double domega = (K_t * i_a - b * omega - T_load) / J;
        double di_a = (v_a - R_a * i_a - K_e * omega) / L_a;

        theta += dtheta * dt;
        omega += domega * dt;
        i_a += di_a * dt;
    }

    public double getTheta() { return theta; }
    public double getOmega() { return omega; }
    public double getCurrent() { return i_a; }

    public static void main(String[] args) {
        DCMotorModel motor = new DCMotorModel();
        double dt = 1e-3;

        for (int k = 0; k < 2000; ++k) {
            motor.step(24.0, 0.0, dt);
        }

        System.out.println("Final omega = " + motor.getOmega());
    }
}
