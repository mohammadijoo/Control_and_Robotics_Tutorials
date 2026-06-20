#include <chrono>
#include <thread>
#include <iostream>

using Clock = std::chrono::steady_clock;

int main() {
    const auto T = std::chrono::milliseconds(5); // 5ms control period
    auto next_release = Clock::now();

    while (true) {
        auto start = Clock::now();

        // --- Task body (bounded WCET assumed) ---
        // read sensors
        // compute control
        // write actuators
        // ---------------------------------------

        auto finish = Clock::now();
        auto exec_time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);

        next_release += T;
        std::this_thread::sleep_until(next_release);

        auto lateness = Clock::now() - next_release;
        std::cout << "Exec(us)=" << exec_time.count()
                  << " lateness(us)="
                  << std::chrono::duration_cast<std::chrono::microseconds>(lateness).count()
                  << std::endl;
    }
    return 0;
}
