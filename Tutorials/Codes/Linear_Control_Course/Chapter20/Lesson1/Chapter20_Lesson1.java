public class TwoDofDemo {
    public static void main(String[] args) {
        double zeta = 0.3;
        double omegaN = 4.0;
        double kPlant = 1.0;

        double Kp1 = 20.0;
        double Ky2 = 20.0;
        double Kr2 = 20.0;
        double Tf = 0.2;

        double dt = 1e-3;
        double tEnd = 2.0;
        int steps = (int) (tEnd / dt);

        double x1Pos = 0.0, x1Vel = 0.0;
        double x2Pos = 0.0, x2Vel = 0.0;
        double r = 1.0;
        double rf = 0.0;

        for (int kStep = 0; kStep != steps; ++kStep) {
            double y1 = x1Pos;
            double y2 = x2Pos;

            double u1 = Kp1 * (r - y1);

            rf += dt * (r - rf) / Tf;
            double u2 = Kr2 * rf - Ky2 * y2;

            double dx1Pos = x1Vel;
            double dx1Vel = -2.0 * zeta * omegaN * x1Vel
                          - omegaN * omegaN * x1Pos
                          + kPlant * u1;
            x1Pos += dt * dx1Pos;
            x1Vel += dt * dx1Vel;

            double dx2Pos = x2Vel;
            double dx2Vel = -2.0 * zeta * omegaN * x2Vel
                          - omegaN * omegaN * x2Pos
                          + kPlant * u2;
            x2Pos += dt * dx2Pos;
            x2Vel += dt * dx2Vel;
        }

        System.out.println("Final 1-DOF position = " + x1Pos);
        System.out.println("Final 2-DOF position = " + x2Pos);
    }
}
