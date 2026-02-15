#include <vector>
#include <limits>
#include <utility>

struct Bid {
    int task;
    double amount;
};

class Robot {
public:
    Robot(int id, const std::vector<double>& v, double eps)
        : id_(id), values_(v), epsilon_(eps), assignedTask_(-1) {}

    bool isUnassigned() const { return assignedTask_ < 0; }

    Bid computeBid(const std::vector<double>& prices) const {
        const double NEG_INF = -std::numeric_limits<double>::infinity();
        double best = NEG_INF, second = NEG_INF;
        int j1 = -1;
        for (int j = 0; j < (int)values_.size(); ++j) {
            double reduced = values_[j] - prices[j];
            if (reduced > best) {
                second = best;
                best = reduced;
                j1 = j;
            } else if (reduced > second) {
                second = reduced;
            }
        }
        double bidAmount = prices[j1] + (best - second) + epsilon_;
        return Bid{j1, bidAmount};
    }

    int id_;
    std::vector<double> values_;
    double epsilon_;
    int assignedTask_;
};
      
