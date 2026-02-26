// Chapter15_Lesson5.java
// Lab: Compare Local Planners in Dense Obstacles (DWA vs simplified TEB-proxy)
// Build: javac Chapter15_Lesson5.java
// Run:   java Chapter15_Lesson5 30 0
import java.util.*;

public class Chapter15_Lesson5 {

  static class Ob { double x,y,r; Ob(double x,double y,double r){this.x=x;this.y=y;this.r=r;} }
  static class Res { boolean success=false, collision=false; double time=0,L=0,cmin=1e9,w2=0; }
  static class Params{
    double R=0.25,vmax=0.9,wmax=1.6,a=1.2,alpha=2.5,dt=0.1,T=2.0,goalTol=0.25;
    int maxSteps=800,nObs=45;
  }

  static double wrap(double a){
    a = (a + Math.PI) % (2*Math.PI);
    if(a < 0) a += 2*Math.PI;
    return a - Math.PI;
  }
  static double hypot(double x,double y){ return Math.hypot(x,y); }
  static double corridorY(double x){ return 0.9*Math.sin(0.6*x); }

  static double[][] referencePath(int n){
    double x0=0.5,x1=11.5;
    double[][] p = new double[n][2];
    for(int i=0;i<n;i++){
      double x = x0 + (x1-x0)*((double)i/(n-1));
      p[i][0]=x; p[i][1]=corridorY(x);
    }
    return p;
  }

  static ArrayList<Ob> sampleDenseWorld(int nObs, int seed){
    Random rng = new Random(seed);
    ArrayList<Ob> obs = new ArrayList<>(nObs);
    int tries=0;
    while(obs.size()<nObs && tries<20000){
      tries++;
      double x = 0.5 + 11.0*rng.nextDouble();
      double y = -2.5 + 5.0*rng.nextDouble();
      double r = 0.12 + 0.20*rng.nextDouble();
      if(Math.abs(y - corridorY(x)) < 0.55 + r) continue;

      boolean ok=true;
      for(Ob o: obs){
        double dx=x-o.x, dy=y-o.y;
        double rr=r+o.r+0.05;
        if(dx*dx+dy*dy < rr*rr){ ok=false; break; }
      }
      if(ok) obs.add(new Ob(x,y,r));
    }
    return obs;
  }

  static double clearance(double px,double py, ArrayList<Ob> obs, double R){
    double d=1e18;
    for(Ob o: obs){
      d = Math.min(d, hypot(px-o.x, py-o.y) - o.r - R);
    }
    return d;
  }

  static double[] stepUnicycle(double[] x, double v,double w,double dt){
    return new double[]{ x[0] + v*Math.cos(x[2])*dt,
                         x[1] + v*Math.sin(x[2])*dt,
                         wrap(x[2] + w*dt) };
  }

  static class Rollout { double[] xf; double minClear; Rollout(double[] xf,double mc){this.xf=xf; this.minClear=mc;} }

  static Rollout rolloutMinClear(double[] x0, double v,double w, ArrayList<Ob> obs, Params p){
    int steps = Math.max(1, (int)Math.round(p.T/p.dt));
    double[] x = x0.clone();
    double mc = 1e18;
    for(int i=0;i<steps;i++){
      x = stepUnicycle(x,v,w,p.dt);
      mc = Math.min(mc, clearance(x[0],x[1],obs,p.R));
      if(mc<=0) break;
    }
    return new Rollout(x,mc);
  }

  static double[] dwaCommand(double[] x, double v0,double w0, double[][] path, ArrayList<Ob> obs, Params p){
    double vmin=Math.max(0.0, v0 - p.a*p.dt), vmax=Math.min(p.vmax, v0 + p.a*p.dt);
    double wmin=Math.max(-p.wmax, w0 - p.alpha*p.dt), wmax=Math.min(p.wmax, w0 + p.alpha*p.dt);
    double bestJ=-1e18, bestV=0, bestW=0;
    double[] goal = path[path.length-1];

    for(int i=0;i<9;i++){
      double v = vmin + (vmax-vmin)*(i/8.0);
      for(int j=0;j<17;j++){
        double w = wmin + (wmax-wmin)*(j/16.0);
        Rollout rr = rolloutMinClear(x,v,w,obs,p);
        double mc = rr.minClear;
        if(mc <= p.R + 0.05) continue;
        double gx=rr.xf[0]-goal[0], gy=rr.xf[1]-goal[1];
        double goalDist = hypot(gx,gy);
        double J = -0.6*goalDist + 1.8*mc + 0.4*(v/(p.vmax+1e-9)) - 0.12*(w*w);
        if(J>bestJ){ bestJ=J; bestV=v; bestW=w; }
      }
    }
    return new double[]{bestV,bestW};
  }

  static double[][] optimizeBand(double[] x, ArrayList<Ob> obs, Params p, int N, double bandLen, int iters, double step){
    double[][] pts = new double[N][2];
    for(int i=0;i<N;i++){
      double xx = x[0] + bandLen*(i/(double)(N-1));
      pts[i][0]=xx; pts[i][1]=corridorY(xx);
    }
    pts[0][0]=x[0]; pts[0][1]=x[1];

    for(int it=0; it<iters; it++){
      double[][] g = new double[N][2];

      for(int i=1;i<N-1;i++){
        g[i][0] += 0.35*(2*pts[i][0] - pts[i-1][0] - pts[i+1][0]);
        g[i][1] += 0.35*(2*pts[i][1] - pts[i-1][1] - pts[i+1][1]);
      }

      for(int i=1;i<N;i++){
        double bestD=1e18, ux=0, uy=0;
        for(Ob o: obs){
          double vx=pts[i][0]-o.x, vy=pts[i][1]-o.y;
          double n = hypot(vx,vy) + 1e-12;
          double d = n - o.r - p.R;
          if(d<bestD){ bestD=d; ux=vx/n; uy=vy/n; }
        }
        if(bestD < 0.9){
          double phi = Math.exp(-4.5*(bestD - 0.9));
          g[i][0] += 1.8*(-4.5*phi)*ux;
          g[i][1] += 1.8*(-4.5*phi)*uy;
        }
      }

      for(int i=1;i<N;i++){
        g[i][0] += 0.25*(pts[i][0]-pts[i-1][0]);
        g[i][1] += 0.25*(pts[i][1]-pts[i-1][1]);
      }

      for(int i=1;i<N;i++){
        pts[i][0] -= step*g[i][0];
        pts[i][1] -= step*g[i][1];
      }
      pts[0][0]=x[0]; pts[0][1]=x[1];
    }
    return pts;
  }

  static double[] tebCommand(double[] x, double v0,double w0, double[][] path, ArrayList<Ob> obs, Params p){
    double[][] pts = optimizeBand(x, obs, p, 12, 2.6, 20, 0.12);
    double dx=pts[1][0]-pts[0][0], dy=pts[1][1]-pts[0][1];
    double heading = Math.atan2(dy,dx);
    double herr = wrap(heading - x[2]);

    double w = Math.max(-p.wmax, Math.min(p.wmax, 2.2*herr));
    double v = Math.max(0.0, Math.min(p.vmax, 0.8*(hypot(dx,dy)/p.dt)));
    v *= 1.0/(1.0 + 1.2*Math.abs(w));

    v = Math.max(Math.max(0.0, v0 - p.a*p.dt), Math.min(Math.min(p.vmax, v0 + p.a*p.dt), v));
    w = Math.max(w0 - p.alpha*p.dt, Math.min(w0 + p.alpha*p.dt, w));
    return new double[]{v,w};
  }

  static Res runOne(String planner, int seed, Params p){
    ArrayList<Ob> obs = sampleDenseWorld(p.nObs, seed);
    double[][] path = referencePath(120);
    double[] goal = path[path.length-1];

    double[] x = new double[]{0.6,0.0,0.0};
    double v=0,w=0;
    Res r = new Res();
    r.time = p.maxSteps*p.dt;

    for(int k=0;k<p.maxSteps;k++){
      double c = clearance(x[0],x[1],obs,p.R);
      r.cmin = Math.min(r.cmin, c);
      if(c<=0){ r.collision=true; r.time=k*p.dt; return r; }
      if(hypot(x[0]-goal[0], x[1]-goal[1]) <= p.goalTol){ r.success=true; r.time=k*p.dt; return r; }

      double[] u = planner.equals("teb") ? tebCommand(x,v,w,path,obs,p) : dwaCommand(x,v,w,path,obs,p);
      double vcmd=u[0], wcmd=u[1];

      double[] x2 = stepUnicycle(x, vcmd, wcmd, p.dt);
      r.L += hypot(x2[0]-x[0], x2[1]-x[1]);
      r.w2 += (wcmd*wcmd)*p.dt;
      x=x2; v=vcmd; w=wcmd;
    }
    return r;
  }

  static Map<String,Double> summarize(ArrayList<Res> R){
    int n=R.size();
    double succ=0,col=0,t=0,L=0,cmin=0,w2=0;
    for(Res r: R){
      succ += r.success?1:0; col += r.collision?1:0;
      t += r.time; L += r.L; cmin += r.cmin; w2 += r.w2;
    }
    LinkedHashMap<String,Double> s = new LinkedHashMap<>();
    s.put("trials", (double)n);
    s.put("success_rate", succ/n);
    s.put("collision_rate", col/n);
    s.put("time_mean", t/n);
    s.put("L_mean", L/n);
    s.put("cmin_mean", cmin/n);
    s.put("w2_mean", w2/n);
    return s;
  }

  public static void main(String[] args){
    int trials = (args.length>0) ? Integer.parseInt(args[0]) : 30;
    int seed0  = (args.length>1) ? Integer.parseInt(args[1]) : 0;
    Params p = new Params();

    for(String name: new String[]{"dwa","teb"}){
      ArrayList<Res> R = new ArrayList<>(trials);
      for(int i=0;i<trials;i++) R.add(runOne(name, seed0+i, p));
      System.out.println("\n" + name.toUpperCase() + " " + summarize(R));
    }
  }
}
