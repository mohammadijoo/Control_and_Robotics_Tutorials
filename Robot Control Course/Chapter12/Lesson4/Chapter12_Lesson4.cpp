
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>

std::atomic<bool> running{true};

void control_loop()
{
    using clock = std::chrono::steady_clock;
    using namespace std::chrono;

    const microseconds period(1000); // 1 kHz
    auto next_time = clock::now();

    while (running.load()) {
        auto start = clock::now();

        // TODO: read sensors, compute control, write actuators here
        // e.g. call into your robot dynamics / controller library

        // measure jitter as difference between actual start and ideal time
        auto jitter = duration_cast<nanoseconds>(start - next_time);
        std::cout << "jitter (ns) = " << jitter.count() << std::endl;

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

int main()
{
    std::thread th(control_loop);

    // In a real system, set real-time priority via sched_setscheduler
    // and pin the thread to a CPU core to reduce jitter.

    std::this_thread::sleep_for(std::chrono::seconds(2));
    running.store(false);
    th.join();
    return 0;
}
