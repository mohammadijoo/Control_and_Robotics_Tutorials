
import org.ejml.simple.SimpleMatrix;

public class ComputedTorqueController {

    // Example placeholders - implement your own dynamics or call a generator
    public SimpleMatrix M(SimpleMatrix q) { /* ... */ return null; }
    public SimpleMatrix C(SimpleMatrix q, SimpleMatrix qd) { /* ... */ return null; }
    public SimpleMatrix g(SimpleMatrix q) { /* ... */ return null; }

    public SimpleMatrix computeTau(
            SimpleMatrix q,
            SimpleMatrix qd,
            SimpleMatrix q_d,
            SimpleMatrix qd_d,
            SimpleMatrix qdd_d,
            SimpleMatrix Kp,
            SimpleMatrix Kd) {

        SimpleMatrix e  = q.minus(q_d);
        SimpleMatrix ed = qd.minus(qd_d);
        SimpleMatrix v  = qdd_d.minus(Kd.mult(ed)).minus(Kp.mult(e));

        SimpleMatrix Mq = M(q);
        SimpleMatrix Cq = C(q, qd);
        SimpleMatrix gq = g(q);

        SimpleMatrix tau = Mq.mult(v).plus(Cq.mult(qd)).plus(gq);
        return tau;
    }
}
