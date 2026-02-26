public class MovingBaseArmSim {

    static class State {
        double x, xdot, q, qdot;
        State(double x, double xdot, double q, double qdot){
            this.x=x; this.xdot=xdot; this.q=q; this.qdot=qdot;
        }
        State copy(){ return new State(x, xdot, q, qdot); }
    }

    // Parameters
    static double I=0.2, M=1.0, m=0.5, ell=0.6, g=9.81;
    static double Kp=15.0, Kd=3.0;

    static State f(State s){
        double tau = -Kp*s.q - Kd*s.qdot;
        double ub  = 0.0;

        double M11 = M + m;
        double M12 = m*ell*Math.cos(s.q);
        double M22 = I + m*ell*ell;

        double det = M11*M22 - M12*M12;

        double h1 = -m*ell*Math.sin(s.q)*s.qdot*s.qdot;
        double h2 =  m*g*ell*Math.sin(s.q);

        double rhs1 = ub - h1;
        double rhs2 = tau - h2;

        // Solve 2x2 system manually
        double xddot = ( M22*rhs1 - M12*rhs2)/det;
        double qddot = (-M12*rhs1 + M11*rhs2)/det;

        return new State(s.xdot, xddot, s.qdot, qddot);
    }

    static State rk4Step(State s, double dt){
        State k1 = f(s);
        State s2 = new State(s.x + 0.5*dt*k1.x, s.xdot + 0.5*dt*k1.xdot,
                             s.q + 0.5*dt*k1.q, s.qdot + 0.5*dt*k1.qdot);
        State k2 = f(s2);
        State s3 = new State(s.x + 0.5*dt*k2.x, s.xdot + 0.5*dt*k2.xdot,
                             s.q + 0.5*dt*k2.q, s.qdot + 0.5*dt*k2.qdot);
        State k3 = f(s3);
        State s4 = new State(s.x + dt*k3.x, s.xdot + dt*k3.xdot,
                             s.q + dt*k3.q, s.qdot + dt*k3.qdot);
        State k4 = f(s4);

        return new State(
            s.x    + dt*(k1.x    + 2*k2.x    + 2*k3.x    + k4.x)/6.0,
            s.xdot + dt*(k1.xdot + 2*k2.xdot + 2*k3.xdot + k4.xdot)/6.0,
            s.q    + dt*(k1.q    + 2*k2.q    + 2*k3.q    + k4.q)/6.0,
            s.qdot + dt*(k1.qdot + 2*k2.qdot + 2*k3.qdot + k4.qdot)/6.0
        );
    }

    public static void main(String[] args){
        State s = new State(0,0,0.5,0);
        double dt=0.001;
        for(int i=0;i<5000;i++){
            s = rk4Step(s, dt);
        }
        System.out.println("Final q=" + s.q + " Final x=" + s.x);
    }
}
      