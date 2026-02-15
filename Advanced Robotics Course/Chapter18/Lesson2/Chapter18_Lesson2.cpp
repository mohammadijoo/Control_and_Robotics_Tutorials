#include <vector>
#include <array>
#include <string>
#include <random>
#include <memory>

struct Box2D {
  double xmin, ymin, xmax, ymax;
};

struct PlanningTask2D {
  std::array<double, 2> start;
  std::array<double, 2> goal;
  std::vector<Box2D> obstacles;
  std::string id;       // unique name for logging
  int stratum_id;       // e.g. clutter level
};

class TaskGenerator {
public:
  explicit TaskGenerator(unsigned seed = 0)
      : gen_(seed), uni_(0.0, 1.0) {}

  PlanningTask2D sampleTask(int numObs, int stratumId, const std::string& prefix) {
    PlanningTask2D t;
    t.start = {rand01(), rand01()};
    t.goal  = {rand01(), rand01()};
    t.stratum_id = stratumId;

    for (int i = 0; i < numObs; ++i) {
      t.obstacles.push_back(randomBox());
    }
    t.id = prefix + "_" + std::to_string(counter_++);
    return t;
  }

private:
  std::mt19937 gen_;
  std::uniform_real_distribution<double> uni_;
  int counter_ = 0;

  double rand01() { return uni_(gen_); }

  Box2D randomBox() {
    double x1 = rand01(), y1 = rand01();
    double x2 = rand01(), y2 = rand01();
    double xmin = std::min(x1, x2);
    double xmax = std::max(x1, x2);
    double ymin = std::min(y1, y2);
    double ymax = std::max(y1, y2);
    double w = (xmax - xmin) * 0.7;
    double h = (ymax - ymin) * 0.7;
    return Box2D{xmin, ymin, xmin + w, ymin + h};
  }
};

// Example usage: generate tasks and serialize them for a ROS/MoveIt benchmark node.
int main() {
  TaskGenerator gen(42);
  std::vector<PlanningTask2D> pool;
  for (int i = 0; i < 200; ++i) {
    int numObs = (i % 4 + 1) * 2;  // vary clutter
    int stratum = i % 3;
    pool.push_back(gen.sampleTask(numObs, stratum, "task"));
  }

  // TODO: compute feature vectors and run same greedy k-center selection as in Python.
  // TODO: save to YAML/JSON for consumption by OMPL or MoveIt.
  return 0;
}
      
