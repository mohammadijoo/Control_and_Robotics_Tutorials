import java.util.Random;

public class MotorTwin {
    static double[] fMotor(double[] x, double V,
                           double J, double b, double Kt,
                           double Ke, double R, double L){
        double omega = x[0], i = x[1];
        double domega = (-b/J)*omega + (Kt/J)*i;
        double di     = (-Ke/L)*omega - (R/L)*i + (1.0/L)*V;
        return new double[]{domega, di};
    }

    public static void main(String[] args){
        double Jt=0.02, bt=0.1, Ktt=0.05, Ket=0.05, Rt=2.0, Lt=0.5;
        double Jh=0.03, bh=0.2, Kth=0.04, Keh=Ket, Rh=Rt, Lh=Lt;

        double dt=0.01, T=5.0;
        int N=(int)(T/dt);

        double[] xTrue={0,0}, xHat={0,0};
        double[] Lobs={30.0, 5.0};

        Random rng=new Random(0);

        double[][] Phi=new double[N-1][2];
        double[] domega=new double[N-1];

        double omegaPrev=0.0;

        for(int k=0;k<N-1;k++){
            double t=k*dt;
            double V=(t>0.5)?6.0:0.0;

            double[] dxTrue=fMotor(xTrue,V,Jt,bt,Ktt,Ket,Rt,Lt);
            xTrue[0]+=dt*dxTrue[0]; xTrue[1]+=dt*dxTrue[1];

            double yMeas=xTrue[0]+0.02*rng.nextGaussian();

            double[] dxHat=fMotor(xHat,V,Jh,bh,Kth,Keh,Rh,Lh);
            xHat[0]+=dt*dxHat[0]; xHat[1]+=dt*dxHat[1];

            double r=yMeas-xHat[0];
            xHat[0]+=dt*Lobs[0]*r; xHat[1]+=dt*Lobs[1]*r;

            if(k>0){
                domega[k-1]=(yMeas-omegaPrev)/dt;
                Phi[k-1][0]=-omegaPrev;
                Phi[k-1][1]=xHat[1];
            }
            omegaPrev=yMeas;
        }

        // Solve normal equations for 2x2 case
        double a11=0,a12=0,a22=0,b1=0,b2=0;
        for(int k=0;k<N-2;k++){
            double p1=Phi[k][0], p2=Phi[k][1];
            a11+=p1*p1; a12+=p1*p2; a22+=p2*p2;
            b1+=p1*domega[k]; b2+=p2*domega[k];
        }
        double det=a11*a22-a12*a12;
        double theta1=( a22*b1-a12*b2)/det;
        double theta2=(-a12*b1+a11*b2)/det;

        System.out.println("theta1=b/J estimate: "+theta1);
        System.out.println("theta2=Kt/J estimate: "+theta2);
    }
}
      
