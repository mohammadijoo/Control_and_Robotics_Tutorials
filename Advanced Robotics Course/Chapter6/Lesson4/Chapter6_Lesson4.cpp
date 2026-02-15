#include <array>
#include <string>
#include <unordered_map>
#include <iostream>

class OneDPOMDP {
public:
    using Vec2 = std::array<double, 2>;

    OneDPOMDP() {
        // Transition matrices (row: s, column: s_prime)
        T_["stay"] = { { {0.9, 0.0},
                       {0.1, 1.0} } };
        T_["step"] = { { {0.1, 0.0},
                       {0.9, 1.0} } };

        // Observation likelihoods Z[o][s_prime]
        Z_["door"]    = { {0.2, 0.9} };
        Z_["no-door"] = { {0.8, 0.1} };

        // Start with uniform belief
        belief_ = { {0.5, 0.5} };
    }

    void applyActionAndObservation(const std::string& a,
                                   const std::string& o) {
        Vec2 b_pred = predict(belief_, a);
        Vec2 b_post = { {
            Z_[o][0] * b_pred[0],
            Z_[o][1] * b_pred[1]
        } };
        normalize(b_post);
        belief_ = b_post;
    }

    const Vec2& belief() const {
        return belief_;
    }

private:
    // T_[a][s][s_prime]
    std::unordered_map<std::string, std::array<std::array<double, 2>, 2>> T_;
    // Z_[o][s_prime]
    std::unordered_map<std::string, Vec2> Z_;
    Vec2 belief_;

    static Vec2 predict(const Vec2& b, const std::string& a) {
        Vec2 out = {{0.0, 0.0}};
        // out[s_prime] = sum_s T[a][s][s_prime] * b[s]
        for (int s = 0; s < 2; ++s) {
            for (int sp = 0; sp < 2; ++sp) {
                out[sp] += T_[a][s][sp] * b[s];
            }
        }
        return out;
    }

    static void normalize(Vec2& v) {
        double sum = v[0] + v[1];
        if (sum == 0.0) {
            v[0] = 0.5;
            v[1] = 0.5;
        } else {
            v[0] /= sum;
            v[1] /= sum;
        }
    }
};

int main() {
    OneDPOMDP pomdp;
    pomdp.applyActionAndObservation("step", "door");
    auto b = pomdp.belief();
    std::cout << "Belief after step+door: "
              << b[0] << ", " << b[1] << std::endl;
    return 0;
}
      
