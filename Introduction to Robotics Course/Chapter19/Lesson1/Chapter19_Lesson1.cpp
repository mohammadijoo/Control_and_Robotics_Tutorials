#include <cstdint>

static constexpr uint8_t  SENSOR_ADDR = 0x1E;     // example I2C address
static constexpr uint8_t  REG_ACCEL_X = 0x28;     // example register
static constexpr float    SCALE       = 0.000598f; // units per LSB
static constexpr float    OFFSET      = 0.05f;     // units
static constexpr float    ALPHA       = 0.9f;      // filter coefficient

float y_accel_x = 0.0f; // filtered value

bool read_register_16(uint8_t dev_addr, uint8_t reg_addr, int16_t &value)
{
    uint8_t buf[2] = {0, 0};
    if (!HAL_I2C_Mem_Read(dev_addr, reg_addr, buf, 2)) {
        return false;
    }
    // Assume sensor sends low byte then high byte
    value = static_cast<int16_t>(
        static_cast<uint16_t>(buf[0]) |
        (static_cast<uint16_t>(buf[1]) << 8)
    );
    return true;
}

void update_accel_x()
{
    int16_t raw = 0;
    if (!read_register_16(SENSOR_ADDR, REG_ACCEL_X, raw)) {
        // handle error (timeout, bus fault, etc.)
        return;
    }

    // Convert to engineering units
    float x_k = SCALE * static_cast<float>(raw) + OFFSET;

    // Exponential smoothing filter
    y_accel_x = ALPHA * y_accel_x + (1.0f - ALPHA) * x_k;
}
      
