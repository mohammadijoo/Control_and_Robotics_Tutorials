// Bare-metal style pseudo-C++ (platform-specific HAL omitted)
#include <cstdint>
#include <cmath>

static volatile uint32_t ms_ticks = 0;

extern "C" void SysTick_Handler() { ms_ticks++; }

uint32_t millis() { return ms_ticks; }

// Heartbeat pin toggle every T ms
void heartbeat_task(uint32_t T_ms) {
    static uint32_t last = 0;
    if (millis() - last >= T_ms) {
        last = millis();
        gpio_toggle(LED1);
    }
}

// First-order low-pass filter: y_k = a y_{k-1} + (1-a) x_k
struct LPF {
    float a, y;
    LPF(float tau, float Ts): y(0.0f) {
        a = std::exp(-Ts / tau);
    }
    float step(float x) {
        y = a * y + (1.0f - a) * x;
        return y;
    }
};

int main() {
    clock_init();
    systick_init(1000); // 1 kHz tick
    uart_init(115200);

    LPF filt(0.05f, 0.001f); // tau=50ms, Ts=1ms

    while (1) {
        heartbeat_task(200);

        float raw = adc_read_channel(0);
        float v = filt.step(raw);

        // Sanity: expected range [0.2, 3.0] volts
        if (v < 0.2f || v > 3.0f) {
            fault_flag_set(SENSOR_RANGE_FAULT);
        }

        protocol_poll(); // handles ping and snapshot commands

        watchdog_kick();
    }
}
