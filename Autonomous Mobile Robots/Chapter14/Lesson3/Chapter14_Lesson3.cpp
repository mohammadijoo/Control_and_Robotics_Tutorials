// Chapter14_Lesson3.cpp
// Autonomous Mobile Robots (AMR) — Chapter 14 Lesson 3
// Behavior Trees / State Machines for Navigation
//
// This is a self-contained, from-scratch minimal BT implementation (no external libs).
// It also contains a simple hierarchical finite-state machine (HFSM) for comparison.
//
// Build (example):
//   g++ -std=c++17 -O2 Chapter14_Lesson3.cpp -o Chapter14_Lesson3
// Run:
//   ./Chapter14_Lesson3

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <variant>
#include <chrono>
#include <thread>
#include <cmath>

enum class Status { SUCCESS, FAILURE, RUNNING };

static const char* to_cstr(Status s) {
  switch (s) {
    case Status::SUCCESS: return "SUCCESS";
    case Status::FAILURE: return "FAILURE";
    default: return "RUNNING";
  }
}

using Value = std::variant<bool, int, double, std::string>;
using Blackboard = std::unordered_map<std::string, Value>;

template <typename T>
T bb_get(const Blackboard& bb, const std::string& key, const T& defval) {
  auto it = bb.find(key);
  if (it == bb.end()) return defval;
  if (auto p = std::get_if<T>(&it->second)) return *p;
  return defval;
}

template <typename T>
void bb_set(Blackboard& bb, const std::string& key, const T& v) {
  bb[key] = v;
}

// -----------------------------
// BT core
// -----------------------------
struct Node {
  std::string name;
  explicit Node(std::string n) : name(std::move(n)) {}
  virtual ~Node() = default;
  virtual Status tick(Blackboard& bb) = 0;
};

struct Condition : Node {
  std::function<bool(const Blackboard&)> fn;
  Condition(std::string n, std::function<bool(const Blackboard&)> f)
    : Node(std::move(n)), fn(std::move(f)) {}
  Status tick(Blackboard& bb) override {
    (void)bb;
    return fn(bb) ? Status::SUCCESS : Status::FAILURE;
  }
};

struct Action : Node {
  std::function<std::string(Blackboard&)> fn;
  Action(std::string n, std::function<std::string(Blackboard&)> f)
    : Node(std::move(n)), fn(std::move(f)) {}
  Status tick(Blackboard& bb) override {
    std::string out = fn(bb);
    for (auto& c : out) c = static_cast<char>(std::tolower(c));
    if (out == "success") return Status::SUCCESS;
    if (out == "failure") return Status::FAILURE;
    return Status::RUNNING;
  }
};

struct Sequence : Node {
  std::vector<Node*> children;
  size_t i = 0; // memory
  Sequence(std::string n, std::vector<Node*> ch)
    : Node(std::move(n)), children(std::move(ch)) {}
  Status tick(Blackboard& bb) override {
    while (i < children.size()) {
      Status s = children[i]->tick(bb);
      if (s == Status::SUCCESS) { ++i; continue; }
      if (s == Status::FAILURE) { i = 0; return Status::FAILURE; }
      return Status::RUNNING;
    }
    i = 0;
    return Status::SUCCESS;
  }
};

struct Fallback : Node {
  std::vector<Node*> children;
  size_t i = 0; // memory
  Fallback(std::string n, std::vector<Node*> ch)
    : Node(std::move(n)), children(std::move(ch)) {}
  Status tick(Blackboard& bb) override {
    while (i < children.size()) {
      Status s = children[i]->tick(bb);
      if (s == Status::FAILURE) { ++i; continue; }
      if (s == Status::SUCCESS) { i = 0; return Status::SUCCESS; }
      return Status::RUNNING;
    }
    i = 0;
    return Status::FAILURE;
  }
};

struct RateLimiter : Node {
  Node* child;
  double dt;
  std::chrono::steady_clock::time_point last;
  Status last_status = Status::FAILURE;

  RateLimiter(std::string n, Node* c, double dt_sec)
    : Node(std::move(n)), child(c), dt(dt_sec), last(std::chrono::steady_clock::now()) {
    last -= std::chrono::milliseconds(static_cast<int>(dt * 1000)); // force first tick
  }

  Status tick(Blackboard& bb) override {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last).count();
    if (elapsed >= dt) {
      last = now;
      last_status = child->tick(bb);
    }
    return last_status;
  }
};

// -----------------------------
// Navigation primitives
// -----------------------------
static bool have_goal(const Blackboard& bb) {
  return bb_get<bool>(bb, "have_goal", false);
}
static bool localization_ok(const Blackboard& bb) {
  double P_trace = bb_get<double>(bb, "P_trace", 1e9);
  double P_max   = bb_get<double>(bb, "P_trace_max", 2.0);
  return P_trace <= P_max;
}
static bool path_valid(const Blackboard& bb) {
  return bb_get<bool>(bb, "path_valid", false);
}
static bool goal_reached(const Blackboard& bb) {
  double d = bb_get<double>(bb, "dist_to_goal", 1e9);
  double tol = bb_get<double>(bb, "goal_tol", 0.2);
  return d <= tol;
}

static std::string global_plan(Blackboard& bb) {
  if (!have_goal(bb)) return "failure";
  if (!localization_ok(bb)) return "failure";
  bb_set(bb, "path_valid", true);
  int calls = bb_get<int>(bb, "planner_calls", 0);
  bb_set(bb, "planner_calls", calls + 1);
  return "success";
}

static std::string local_control(Blackboard& bb) {
  if (!path_valid(bb)) return "failure";
  bool obs = bb_get<bool>(bb, "obstacle_blocking", false);
  if (obs) return "running";

  double d = bb_get<double>(bb, "dist_to_goal", 5.0);
  double step = bb_get<double>(bb, "progress_per_tick", 0.2);
  d = std::max(0.0, d - step);
  bb_set(bb, "dist_to_goal", d);
  return goal_reached(bb) ? "success" : "running";
}

static std::string recover_clear_costmap(Blackboard& bb) {
  bb_set(bb, "obstacle_blocking", false);
  bb_set(bb, "path_valid", false);
  int rec = bb_get<int>(bb, "recoveries", 0);
  bb_set(bb, "recoveries", rec + 1);
  return "success";
}

// -----------------------------
// HFSM
// -----------------------------
enum class NavState { IDLE, PLAN, CONTROL, RECOVERY, DONE, FAIL };

static const char* to_cstr(NavState s) {
  switch (s) {
    case NavState::IDLE: return "IDLE";
    case NavState::PLAN: return "PLAN";
    case NavState::CONTROL: return "CONTROL";
    case NavState::RECOVERY: return "RECOVERY";
    case NavState::DONE: return "DONE";
    default: return "FAIL";
  }
}

struct HFSM {
  NavState st = NavState::IDLE;
  int stall = 0;
  int stall_max = 10;

  NavState step(Blackboard& bb) {
    if (st == NavState::IDLE) {
      st = (have_goal(bb) && localization_ok(bb)) ? NavState::PLAN : NavState::FAIL;
      return st;
    }
    if (st == NavState::PLAN) {
      st = (global_plan(bb) == "success") ? NavState::CONTROL : NavState::FAIL;
      return st;
    }
    if (st == NavState::CONTROL) {
      if (goal_reached(bb)) { st = NavState::DONE; return st; }
      std::string out = local_control(bb);
      if (out == "failure") { st = NavState::PLAN; return st; }
      bool obs = bb_get<bool>(bb, "obstacle_blocking", false);
      if (obs) {
        ++stall;
        if (stall >= stall_max) { st = NavState::RECOVERY; stall = 0; }
      } else stall = 0;
      return st;
    }
    if (st == NavState::RECOVERY) {
      recover_clear_costmap(bb);
      st = NavState::PLAN;
      return st;
    }
    return st;
  }
};

// -----------------------------
// Build BT
// -----------------------------
Node* build_navigation_bt(std::vector<std::unique_ptr<Node>>& arena) {
  auto make = [&](auto ptr) -> Node* {
    arena.emplace_back(std::move(ptr));
    return arena.back().get();
  };

  Node* haveGoal = make(std::make_unique<Condition>("HaveGoal?", [](const Blackboard& bb){ return have_goal(bb); }));
  Node* locOK    = make(std::make_unique<Condition>("LocalizationOK?", [](const Blackboard& bb){ return localization_ok(bb); }));
  Node* pathOK   = make(std::make_unique<Condition>("PathValid?", [](const Blackboard& bb){ return path_valid(bb); }));
  Node* reached  = make(std::make_unique<Condition>("GoalReached?", [](const Blackboard& bb){ return goal_reached(bb); }));

  Node* planAct  = make(std::make_unique<Action>("GlobalPlan", [](Blackboard& bb){ return global_plan(bb); }));
  Node* planRate = make(std::make_unique<RateLimiter>("Replan@1Hz", planAct, 1.0));

  Node* ctrlAct  = make(std::make_unique<Action>("LocalControl", [](Blackboard& bb){ return local_control(bb); }));
  Node* recAct   = make(std::make_unique<Action>("RecoveryClearCostmap", [](Blackboard& bb){ return recover_clear_costmap(bb); }));

  Node* driveSeq = make(std::make_unique<Sequence>("DriveToGoal", std::vector<Node*>{ pathOK, ctrlAct, reached }));
  Node* driveOrRecover = make(std::make_unique<Fallback>("DriveOrRecover", std::vector<Node*>{ driveSeq, recAct }));

  Node* root = make(std::make_unique<Sequence>("NavigateToGoal",
    std::vector<Node*>{ haveGoal, locOK, planRate, driveOrRecover }));

  return root;
}

int main() {
  Blackboard bb;
  bb_set(bb, "have_goal", true);
  bb_set(bb, "dist_to_goal", 5.0);
  bb_set(bb, "goal_tol", 0.2);
  bb_set(bb, "P_trace", 0.8);
  bb_set(bb, "P_trace_max", 2.0);
  bb_set(bb, "progress_per_tick", 0.2);
  bb_set(bb, "obstacle_blocking", false);
  bb_set(bb, "path_valid", false);
  bb_set(bb, "recoveries", 0);
  bb_set(bb, "planner_calls", 0);

  std::cout << "=== Behavior Tree demo ===\n";
  std::vector<std::unique_ptr<Node>> arena;
  Node* bt = build_navigation_bt(arena);

  for (int t = 0; t < 60; ++t) {
    if (t == 12) bb_set(bb, "obstacle_blocking", true);
    if (t == 25) bb_set(bb, "obstacle_blocking", false);

    Status s = bt->tick(bb);
    std::cout << "t=" << (t<10?"0":"") << t
              << " status=" << to_cstr(s)
              << " dist=" << bb_get<double>(bb, "dist_to_goal", 0.0)
              << " path_valid=" << (bb_get<bool>(bb, "path_valid", false) ? "true":"false")
              << " rec=" << bb_get<int>(bb, "recoveries", 0)
              << "\n";
    if (s == Status::SUCCESS) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Reset and run HFSM
  bb_set(bb, "dist_to_goal", 5.0);
  bb_set(bb, "path_valid", false);
  bb_set(bb, "obstacle_blocking", false);
  bb_set(bb, "recoveries", 0);
  bb_set(bb, "planner_calls", 0);

  std::cout << "\n=== HFSM demo ===\n";
  HFSM fsm;
  for (int t = 0; t < 60; ++t) {
    if (t == 12) bb_set(bb, "obstacle_blocking", true);
    if (t == 25) bb_set(bb, "obstacle_blocking", false);

    NavState st = fsm.step(bb);
    std::cout << "t=" << (t<10?"0":"") << t
              << " state=" << to_cstr(st)
              << " dist=" << bb_get<double>(bb, "dist_to_goal", 0.0)
              << " path_valid=" << (bb_get<bool>(bb, "path_valid", false) ? "true":"false")
              << " rec=" << bb_get<int>(bb, "recoveries", 0)
              << "\n";
    if (st == NavState::DONE) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return 0;
}
