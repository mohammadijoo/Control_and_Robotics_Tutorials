public class DCMotorPID {
    static class DCMotor {
        double R, L, ke, kt, J, b;
        double i = 0.0, omega = 0.0;
        DCMotor(double R,double L,double ke,double J,double b){
            this.R=R; this.L=L; this.ke=ke; this.kt=ke; this.J=J; this.b=b;
        }
        void step(double v, double dt){
            double di = (v - R*i - ke*omega)/L;
            i += di*dt;
            double domega = (kt*i - b*omega)/J;
            omega += domega*dt;
        }
    }

    static class PID {
        double Kp, Ki, Kd, eint=0.0, eprev=0.0;
        PID(double Kp,double Ki,double Kd){ this.Kp=Kp; this.Ki=Ki; this.Kd=Kd; }
        double update(double e, double dt){
            eint += e*dt;
            double eder = (e-eprev)/dt;
            eprev = e;
            return Kp*e + Ki*eint + Kd*eder;
        }
    }

    public static void main(String[] args){
        DCMotor m = new DCMotor(1.2, 2e-3, 0.08, 5e-4, 1e-3);
        PID pid = new PID(0.4, 30.0, 0.0);

        double dt=1e-4, Tend=0.5;
        int N=(int)(Tend/dt);
        double omegaRef=100.0;

        for(int k=0; k<N; k++){
            double e = omegaRef - m.omega;
            double v = pid.update(e, dt);
            m.step(v, dt);
            if(k % 1000 == 0){
                System.out.println((k*dt) + "," + m.omega);
            }
        }
    }
}
