// Chapter14_Lesson3.java
// Autonomous Mobile Robots (AMR) — Chapter 14 Lesson 3
// Behavior Trees / State Machines for Navigation
//
// A single-file Java implementation (all classes as static inner classes).
//
// Compile:
//   javac Chapter14_Lesson3.java
// Run:
//   java Chapter14_Lesson3

import java.util.*;

public class Chapter14_Lesson3 {

  // -----------------------------
  // Behavior Tree core
  // -----------------------------
  enum Status { SUCCESS, FAILURE, RUNNING }

  interface Node {
    Status tick(Map<String, Object> bb);
    String name();
  }

  static final class Condition implements Node {
    private final String name;
    private final java.util.function.Predicate<Map<String,Object>> fn;
    Condition(String name, java.util.function.Predicate<Map<String,Object>> fn) {
      this.name = name; this.fn = fn;
    }
    public Status tick(Map<String,Object> bb) { return fn.test(bb) ? Status.SUCCESS : Status.FAILURE; }
    public String name() { return name; }
  }

  static final class Action implements Node {
    private final String name;
    private final java.util.function.Function<Map<String,Object>, String> fn;
    Action(String name, java.util.function.Function<Map<String,Object>, String> fn) {
      this.name = name; this.fn = fn;
    }
    public Status tick(Map<String,Object> bb) {
      String out = fn.apply(bb).toLowerCase(Locale.ROOT).trim();
      if (out.equals("success")) return Status.SUCCESS;
      if (out.equals("failure")) return Status.FAILURE;
      return Status.RUNNING;
    }
    public String name() { return name; }
  }

  static final class Sequence implements Node {
    private final String name;
    private final List<Node> children;
    private int i = 0; // memory
    Sequence(String name, List<Node> children) { this.name = name; this.children = children; }
    public Status tick(Map<String,Object> bb) {
      while (i < children.size()) {
        Status s = children.get(i).tick(bb);
        if (s == Status.SUCCESS) { i++; continue; }
        if (s == Status.FAILURE) { i = 0; return Status.FAILURE; }
        return Status.RUNNING;
      }
      i = 0;
      return Status.SUCCESS;
    }
    public String name() { return name; }
  }

  static final class Fallback implements Node {
    private final String name;
    private final List<Node> children;
    private int i = 0; // memory
    Fallback(String name, List<Node> children) { this.name = name; this.children = children; }
    public Status tick(Map<String,Object> bb) {
      while (i < children.size()) {
        Status s = children.get(i).tick(bb);
        if (s == Status.FAILURE) { i++; continue; }
        if (s == Status.SUCCESS) { i = 0; return Status.SUCCESS; }
        return Status.RUNNING;
      }
      i = 0;
      return Status.FAILURE;
    }
    public String name() { return name; }
  }

  static final class RateLimiter implements Node {
    private final String name;
    private final Node child;
    private final long dtMillis;
    private long last = 0L;
    private Status lastStatus = Status.FAILURE;
    RateLimiter(String name, Node child, double dtSeconds) {
      this.name = name; this.child = child; this.dtMillis = (long)(dtSeconds * 1000.0);
      this.last = 0L;
    }
    public Status tick(Map<String,Object> bb) {
      long now = System.currentTimeMillis();
      if (now - last >= dtMillis) {
        last = now;
        lastStatus = child.tick(bb);
      }
      return lastStatus;
    }
    public String name() { return name; }
  }

  // -----------------------------
  // Navigation primitives
  // -----------------------------
  static boolean haveGoal(Map<String,Object> bb) { return (Boolean)bb.getOrDefault("have_goal", false); }

  static boolean localizationOk(Map<String,Object> bb) {
    double P = (Double)bb.getOrDefault("P_trace", 1e9);
    double Pmax = (Double)bb.getOrDefault("P_trace_max", 2.0);
    return P <= Pmax;
  }

  static boolean pathValid(Map<String,Object> bb) { return (Boolean)bb.getOrDefault("path_valid", false); }

  static boolean goalReached(Map<String,Object> bb) {
    double d = (Double)bb.getOrDefault("dist_to_goal", 1e9);
    double tol = (Double)bb.getOrDefault("goal_tol", 0.2);
    return d <= tol;
  }

  static String globalPlan(Map<String,Object> bb) {
    if (!haveGoal(bb)) return "failure";
    if (!localizationOk(bb)) return "failure";
    bb.put("path_valid", true);
    int calls = (Integer)bb.getOrDefault("planner_calls", 0);
    bb.put("planner_calls", calls + 1);
    return "success";
  }

  static String localControl(Map<String,Object> bb) {
    if (!pathValid(bb)) return "failure";
    boolean obs = (Boolean)bb.getOrDefault("obstacle_blocking", false);
    if (obs) return "running";

    double d = (Double)bb.getOrDefault("dist_to_goal", 5.0);
    double step = (Double)bb.getOrDefault("progress_per_tick", 0.2);
    d = Math.max(0.0, d - step);
    bb.put("dist_to_goal", d);
    return goalReached(bb) ? "success" : "running";
  }

  static String recoverClearCostmap(Map<String,Object> bb) {
    bb.put("obstacle_blocking", false);
    bb.put("path_valid", false);
    int rec = (Integer)bb.getOrDefault("recoveries", 0);
    bb.put("recoveries", rec + 1);
    return "success";
  }

  static Node buildNavigationBT() {
    Node haveGoal = new Condition("HaveGoal?", Chapter14_Lesson3::haveGoal);
    Node locOK = new Condition("LocalizationOK?", Chapter14_Lesson3::localizationOk);
    Node pathOK = new Condition("PathValid?", Chapter14_Lesson3::pathValid);
    Node reached = new Condition("GoalReached?", Chapter14_Lesson3::goalReached);

    Node planAct = new Action("GlobalPlan", Chapter14_Lesson3::globalPlan);
    Node planRate = new RateLimiter("Replan@1Hz", planAct, 1.0);

    Node ctrlAct = new Action("LocalControl", Chapter14_Lesson3::localControl);
    Node recAct  = new Action("RecoveryClearCostmap", Chapter14_Lesson3::recoverClearCostmap);

    Node driveSeq = new Sequence("DriveToGoal", Arrays.asList(pathOK, ctrlAct, reached));
    Node driveOrRecover = new Fallback("DriveOrRecover", Arrays.asList(driveSeq, recAct));

    return new Sequence("NavigateToGoal", Arrays.asList(haveGoal, locOK, planRate, driveOrRecover));
  }

  // -----------------------------
  // HFSM
  // -----------------------------
  enum NavState { IDLE, PLAN, CONTROL, RECOVERY, DONE, FAIL }

  static final class HFSM {
    NavState st = NavState.IDLE;
    int stall = 0;
    int stallMax = 10;

    NavState step(Map<String,Object> bb) {
      switch (st) {
        case IDLE:
          st = (haveGoal(bb) && localizationOk(bb)) ? NavState.PLAN : NavState.FAIL;
          return st;
        case PLAN:
          st = globalPlan(bb).equals("success") ? NavState.CONTROL : NavState.FAIL;
          return st;
        case CONTROL:
          if (goalReached(bb)) { st = NavState.DONE; return st; }
          String out = localControl(bb);
          if (out.equals("failure")) { st = NavState.PLAN; return st; }
          boolean obs = (Boolean)bb.getOrDefault("obstacle_blocking", false);
          if (obs) {
            stall++;
            if (stall >= stallMax) { stall = 0; st = NavState.RECOVERY; }
          } else stall = 0;
          return st;
        case RECOVERY:
          recoverClearCostmap(bb);
          st = NavState.PLAN;
          return st;
        default:
          return st;
      }
    }
  }

  // -----------------------------
  // Demo
  // -----------------------------
  public static void main(String[] args) throws Exception {
    Map<String,Object> bb = new HashMap<>();
    bb.put("have_goal", true);
    bb.put("dist_to_goal", 5.0);
    bb.put("goal_tol", 0.2);
    bb.put("P_trace", 0.8);
    bb.put("P_trace_max", 2.0);
    bb.put("progress_per_tick", 0.2);
    bb.put("obstacle_blocking", false);
    bb.put("path_valid", false);
    bb.put("recoveries", 0);
    bb.put("planner_calls", 0);

    Node bt = buildNavigationBT();
    System.out.println("=== Behavior Tree demo ===");
    for (int t = 0; t < 60; t++) {
      if (t == 12) bb.put("obstacle_blocking", true);
      if (t == 25) bb.put("obstacle_blocking", false);

      Status s = bt.tick(bb);
      System.out.printf(Locale.ROOT,
        "t=%02d status=%-7s dist=%.2f path_valid=%s rec=%d%n",
        t, s.name(),
        (Double)bb.get("dist_to_goal"),
        bb.get("path_valid"),
        (Integer)bb.get("recoveries")
      );
      if (s == Status.SUCCESS) break;
      Thread.sleep(20);
    }

    // reset and run HFSM
    bb.put("dist_to_goal", 5.0);
    bb.put("path_valid", false);
    bb.put("obstacle_blocking", false);
    bb.put("recoveries", 0);
    bb.put("planner_calls", 0);

    HFSM fsm = new HFSM();
    System.out.println("\n=== HFSM demo ===");
    for (int t = 0; t < 60; t++) {
      if (t == 12) bb.put("obstacle_blocking", true);
      if (t == 25) bb.put("obstacle_blocking", false);

      NavState st = fsm.step(bb);
      System.out.printf(Locale.ROOT,
        "t=%02d state=%-9s dist=%.2f path_valid=%s rec=%d%n",
        t, st.name(),
        (Double)bb.get("dist_to_goal"),
        bb.get("path_valid"),
        (Integer)bb.get("recoveries")
      );
      if (st == NavState.DONE) break;
      Thread.sleep(20);
    }
  }
}
