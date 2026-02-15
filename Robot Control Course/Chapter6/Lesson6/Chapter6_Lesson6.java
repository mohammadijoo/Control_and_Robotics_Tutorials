
public class HybridPegInHoleController {

    // Position gains (for y)
    private double kpy = 2000.0;
    private double kdy = 60.0;

    // Force gains (for x and theta)
    private double kfx = 0.3;
    private double kftheta = 0.2;

    // Desired reference
    private double yDes;
    private double fnDes;      // desired normal force along x
    private double mDes;       // desired moment around theta (often 0)

    public HybridPegInHoleController(double yDes, double fnDes) {
        this.yDes = yDes;
        this.fnDes = fnDes;
        this.mDes = 0.0;
    }

    // Input state and sensor readings:
    // z = [x, y, theta]
    // zd = [xd, yd, thetad]
    // dz = time derivative of z
    // fext = [Fx, Fy, Mz] Cartesian wrench in local peg frame
    //
    // Output: desired Cartesian wrench [Fx_cmd, Fy_cmd, Mz_cmd]
    public double[] computeWrench(double[] z, double[] zd, double[] dz, double[] fext) {
        double x = z[0];
        double y = z[1];
        double theta = z[2];

        double xd = zd[0];
        double yd = zd[1];
        double thetad = zd[2];

        double dx = dz[0];
        double dy = dz[1];
        double dtheta = dz[2];

        double fx = fext[0];
        double fy = fext[1];
        double mz = fext[2];

        // Position error in y
        double ey = y - yd;
        double dey = dy - 0.0;

        // Position control on y (insertion direction)
        double Fy_cmd = -kpy * ey - kdy * dey;

        // Force error in x
        double efx = fx - fnDes;
        // Force error in moment (around theta)
        double efm = mz - mDes;

        // Force control on x and theta
        double Fx_cmd = -kfx * efx;
        double Mz_cmd = -kftheta * efm;

        return new double[]{Fx_cmd, Fy_cmd, Mz_cmd};
    }

    public void setPositionGains(double kpy, double kdy) {
        this.kpy = kpy;
        this.kdy = kdy;
    }

    public void setForceGains(double kfx, double kftheta) {
        this.kfx = kfx;
        this.kftheta = kftheta;
    }
}
