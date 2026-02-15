#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

struct Task {
    std::string name;
    double duration;
    std::vector<std::string> predecessors;
};

using TaskMap = std::unordered_map<std::string, Task>;
using TimeMap = std::unordered_map<std::string, double>;

TimeMap earliest_finish_times(const TaskMap& tasks,
                              const std::vector<std::string>& topo_order)
{
    TimeMap finish, start;
    for (const auto& name : topo_order) {
        const auto& t = tasks.at(name);
        if (t.predecessors.empty()) {
            start[name] = 0.0;
        } else {
            double s = 0.0;
            for (const auto& p : t.predecessors) {
                s = std::max(s, finish[p]);
            }
            start[name] = s;
        }
        finish[name] = start[name] + t.duration;
    }
    return finish;
}

double milestone_time(const TimeMap& finish,
                      const std::unordered_set<std::string>& milestone_tasks)
{
    double T = 0.0;
    for (const auto& name : milestone_tasks) {
        auto it = finish.find(name);
        if (it == finish.end()) {
            throw std::runtime_error("Unknown task in milestone");
        }
        if (it->second > T) {
            T = it->second;
        }
    }
    return T;
}

int main() {
    TaskMap tasks;
    tasks["chassis"] = {"chassis", 8.0, {}};
    tasks["motor_wiring"] = {"motor_wiring", 6.0, {"chassis"}};
    tasks["mcu_bringup"] = {"mcu_bringup", 5.0, {}};
    tasks["low_voltage_test"] =
        {"low_voltage_test", 4.0, {"motor_wiring", "mcu_bringup"}};

    std::vector<std::string> topo_order = {
        "chassis", "mcu_bringup", "motor_wiring", "low_voltage_test"
    };

    TimeMap finish = earliest_finish_times(tasks, topo_order);
    std::unordered_set<std::string> M1 = {
        "chassis", "motor_wiring", "mcu_bringup", "low_voltage_test"
    };

    double T = milestone_time(finish, M1);
    std::cout << "M1 earliest completion time: " << T << std::endl;
    return 0;
}
      
