// Minimal embedded-style C++ (no dynamic allocation)
enum class Mode { OFF, BOOT, IDLE, ACTIVE, FAULT };

struct Telemetry {
  float vbus;  // volts
  float ibus;  // amps
  float temp;  // deg C
  bool  estop;
};

const float VMIN = 20.0f;
const float IMAX = 15.0f;
const float TMAX = 80.0f;

bool safeSet(const Telemetry& z) {
  return (z.vbus >= VMIN) && (z.ibus <= IMAX) && (z.temp <= TMAX) && (!z.estop);
}

Mode supervisorStep(Mode m, const Telemetry& z, bool user_enable) {
  if (!safeSet(z)) return Mode::FAULT;

  switch (m) {
    case Mode::OFF:
      return (z.vbus >= VMIN) ? Mode::BOOT : Mode::OFF;

    case Mode::BOOT:
      // do self-test elsewhere
      return Mode::IDLE;

    case Mode::IDLE:
      return user_enable ? Mode::ACTIVE : Mode::IDLE;

    case Mode::ACTIVE:
      return user_enable ? Mode::ACTIVE : Mode::IDLE;

    case Mode::FAULT:
      return Mode::OFF; // latched shutdown
  }
  return Mode::FAULT;
}

void setDriverEnable(bool en); // hardware gate

void loop() {
  static Mode mode = Mode::OFF;
  Telemetry z = readTelemetry();  // ADC + GPIO sampling
  bool user_enable = readUserEnable();

  mode = supervisorStep(mode, z, user_enable);
  setDriverEnable(mode == Mode::ACTIVE);
}
