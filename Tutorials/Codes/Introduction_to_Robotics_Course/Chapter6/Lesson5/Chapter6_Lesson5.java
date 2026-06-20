public class SEA {
    double Jm, Jl, bm, bl, N, ks;
    // x = [theta_m, omega_m, theta_l, omega_l]
    double[] x = new double[4];

    public SEA(double Jm, double Jl, double bm, double bl, double N, double ks){
        this.Jm=Jm; this.Jl=Jl; this.bm=bm; this.bl=bl; this.N=N; this.ks=ks;
    }

    public double springTorque(){
        double theta_s = N*x[0] - x[2];
        return ks*theta_s;
    }

    public void step(double tau_m, double dt){
        double tau_s = springTorque();

        double domega_m = (tau_m - bm*x[1] - N*tau_s) / Jm;
        double domega_l = (tau_s - bl*x[3]) / Jl;

        x[1] += dt*domega_m;
        x[0] += dt*x[1];
        x[3] += dt*domega_l;
        x[2] += dt*x[3];
    }

    public static void main(String[] args){
        SEA sea = new SEA(0.01,0.05,0.02,0.05,50.0,200.0);
        double Kp=5, Ki=40, Kd=0.02;
        double dt=1e-3, T=2.0;
        int steps = (int)(T/dt);
        double eint=0, eprev=0;

        for(int k=0;k<steps;k++){
            double t=k*dt;
            double tau_d = (t>0.2)?2.0:0.0;
            double tau_s = sea.springTorque();
            double e = tau_d - tau_s;
            eint += e*dt;
            double de = (e-eprev)/dt; eprev=e;
            double tau_m = Kp*e + Ki*eint + Kd*de;
            sea.step(tau_m, dt);
        }
        System.out.println("Final torque: " + sea.springTorque());
    }
}
