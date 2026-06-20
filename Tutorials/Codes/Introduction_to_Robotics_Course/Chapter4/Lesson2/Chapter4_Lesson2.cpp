
#include <iostream>
#include <cmath>

// Example fixed-step control loop for a motor joint.
// In practice, readEncoder() and writePWM() are hardware drivers.

double readEncoderRadians();   // returns quantized angle
void writePWM(double duty);    // duty in [-1,1]

int main() {
    const double Ts  = 0.001;   // 1 kHz loop
    const double Kp  = 2.0;
    const double Kd  = 0.02;
    const double ref = 1.0;

    double theta_prev = 0.0;

    while(true) {
        double theta = readEncoderRadians();
        double omega = (theta - theta_prev)/Ts;

        double e = ref - theta;
        double u = Kp*e - Kd*omega;

        // Convert voltage-like command to PWM duty and saturate
        double duty = std::max(-1.0, std::min(1.0, u/12.0));
        writePWM(duty);

        theta_prev = theta;

        // wait Ts seconds using a real-time timer (platform-specific)
    }
    return 0;
}
      