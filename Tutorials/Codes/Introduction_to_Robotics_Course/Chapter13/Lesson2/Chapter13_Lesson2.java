class Vec2 {
    public double x, y;
    Vec2(double x, double y){ this.x=x; this.y=y; }
    Vec2 add(Vec2 o){ return new Vec2(x+o.x, y+o.y); }
    Vec2 sub(Vec2 o){ return new Vec2(x-o.x, y-o.y); }
    Vec2 mul(double s){ return new Vec2(s*x, s*y); }
    double dot(Vec2 o){ return x*o.x + y*o.y; }
    double norm(){ return Math.sqrt(x*x + y*y); }
    Vec2 unit(){ double n=norm(); return new Vec2(x/n, y/n); }
}

public class DiskCollision {
    static void collide(
        double m1, Vec2 v1,
        double m2, Vec2 v2,
        Vec2 n, double restitution
    ){
        // relative normal speed
        double vn = v1.sub(v2).dot(n);
        if (vn >= 0) return; // separating

        double j = -(1.0 + restitution) * vn / (1.0/m1 + 1.0/m2);
        Vec2 impulse = n.mul(j);

        // apply impulses
        v1.x += impulse.x / m1;  v1.y += impulse.y / m1;
        v2.x -= impulse.x / m2;  v2.y -= impulse.y / m2;
    }
}
      
