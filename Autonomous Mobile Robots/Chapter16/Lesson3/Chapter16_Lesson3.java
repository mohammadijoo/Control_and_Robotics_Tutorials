// Chapter16_Lesson3.java
// Socially aware local velocity selection (sampling-based) for a point robot.
// Output: CSV trajectories for robot and humans.
// Compile: javac Chapter16_Lesson3.java
// Run:     java Chapter16_Lesson3

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Chapter16_Lesson3 {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y){ this.x=x; this.y=y; }
        Vec2 add(Vec2 o){ return new Vec2(x+o.x, y+o.y); }
        Vec2 sub(Vec2 o){ return new Vec2(x-o.x, y-o.y); }
        Vec2 mul(double s){ return new Vec2(x*s, y*s); }
        double dot(Vec2 o){ return x*o.x + y*o.y; }
        double norm2(){ return dot(this); }
        double norm(){ return Math.sqrt(norm2()); }
    }

    static class Human {
        Vec2 p, v;
        Human(Vec2 p, Vec2 v){ this.p=p; this.v=v; }
    }

    static Vec2 rotT(Vec2 r, double th){
        double c = Math.cos(th), s = Math.sin(th);
        return new Vec2(c*r.x + s*r.y, -s*r.x + c*r.y);
    }

    static double anisotropicGaussianCost(Vec2 x, List<Human> humans,
                                          double sigmaFront, double sigmaSide, double sigmaBack){
        double cost = 0.0;
        for(Human h: humans){
            double spd = h.v.norm();
            double th = (spd > 1e-6) ? Math.atan2(h.v.y, h.v.x) : 0.0;
            Vec2 r = x.sub(h.p);
            Vec2 rh = rotT(r, th);
            double sx = (rh.x >= 0.0) ? sigmaFront : sigmaBack;
            double sy = sigmaSide;
            double q = (rh.x*rh.x)/(sx*sx) + (rh.y*rh.y)/(sy*sy);
            cost += Math.exp(-0.5*q);
        }
        return cost;
    }

    static boolean collisionWithinTau(Vec2 vRobot, Vec2 pRobot, List<Human> humans, double R, double tau){
        for(Human h: humans){
            Vec2 p = h.p.sub(pRobot);
            Vec2 vrel = vRobot.sub(h.v);
            double vv = vrel.norm2();
            if(vv < 1e-12){
                if(p.norm() < R) return true;
                continue;
            }
            double tstar = -p.dot(vrel)/vv;
            if(tstar < 0.0) tstar = 0.0;
            if(tstar > tau) tstar = tau;
            Vec2 closest = p.sub(vrel.mul(tstar));
            if(closest.norm() < R) return true;
        }
        return false;
    }

    static List<Vec2> sampleVelocities(Vec2 vPref, double vMax, int nSpeed, int nAngle, double spreadDeg){
        double spref = vPref.norm();
        double th0 = (spref > 1e-8) ? Math.atan2(vPref.y, vPref.x) : 0.0;
        double spread = spreadDeg * Math.PI / 180.0;

        List<Vec2> V = new ArrayList<>();
        for(int i=0;i<nSpeed;i++){
            double s = (double)i/(nSpeed-1) * vMax;
            for(int j=0;j<nAngle;j++){
                double a = -spread + 2.0*spread*(double)j/(nAngle-1);
                double th = th0 + a;
                V.add(new Vec2(s*Math.cos(th), s*Math.sin(th)));
            }
        }
        return V;
    }

    static Vec2 chooseVelocity(Vec2 pRobot, Vec2 vPref, List<Human> humans,
                              double vMax, double wTrack, double wSocial, double wClear,
                              double R, double tau){
        double bestJ = Double.POSITIVE_INFINITY;
        Vec2 bestV = new Vec2(0.0, 0.0);
        double dtEval = 0.6;
        double eps = 1e-3;

        for(Vec2 v: sampleVelocities(vPref, vMax, 9, 21, 80.0)){
            if(collisionWithinTau(v, pRobot, humans, R, tau)) continue;

            Vec2 pNext = pRobot.add(v.mul(dtEval));
            double C = anisotropicGaussianCost(pNext, humans, 1.2, 0.6, 0.8);
            double clear = 0.0;
            for(Human h: humans){
                double d = pNext.sub(h.p).norm();
                clear += 1.0/(d + eps);
            }
            Vec2 dv = v.sub(vPref);
            double J = wTrack*dv.norm2() + wSocial*C + wClear*clear;
            if(J < bestJ){
                bestJ = J;
                bestV = v;
            }
        }
        return bestV;
    }

    public static void main(String[] args) throws IOException {
        double dt = 0.1;
        double T = 26.0;
        int steps = (int)Math.round(T/dt);

        Vec2 p = new Vec2(0.0, 0.0);
        Vec2 goal = new Vec2(10.0, 0.0);
        double vMax = 1.2;

        List<Human> humans = new ArrayList<>();
        humans.add(new Human(new Vec2(4.5,  1.2), new Vec2( 0.35, -0.05)));
        humans.add(new Human(new Vec2(4.0, -1.4), new Vec2( 0.35,  0.08)));
        humans.add(new Human(new Vec2(7.0,  0.3), new Vec2(-0.25,  0.02)));

        try(FileWriter fR = new FileWriter("out_robot.csv");
            FileWriter fH = new FileWriter("out_humans.csv")){

            fR.write("t,x,y\n");
            fH.write("t,id,x,y\n");

            for(int k=0;k<steps;k++){
                double t = k*dt;

                fR.write(t + "," + p.x + "," + p.y + "\n");
                for(int i=0;i<humans.size();i++){
                    Human h = humans.get(i);
                    fH.write(t + "," + (i+1) + "," + h.p.x + "," + h.p.y + "\n");
                }

                for(Human h: humans){
                    h.p = h.p.add(h.v.mul(dt));
                }

                Vec2 e = goal.sub(p);
                double dist = e.norm();
                if(dist < 0.15) break;

                Vec2 dir = e.mul(1.0/(dist + 1e-12));
                double s = Math.min(vMax, 0.8*dist);
                Vec2 vPref = dir.mul(s);

                Vec2 vCmd = chooseVelocity(p, vPref, humans, vMax, 1.0, 1.5, 2.0, 0.55, 3.0);
                p = p.add(vCmd.mul(dt));
            }
        }
        System.out.println("Wrote out_robot.csv and out_humans.csv");
    }
}
