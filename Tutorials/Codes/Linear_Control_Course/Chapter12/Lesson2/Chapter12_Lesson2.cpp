#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

int main() {
    using std::complex;
    using std::cout;
    using std::endl;

    const double K = 20.0;  // plant gain
    const double T = 0.1;   // plant time constant

    // PID parameters obtained from analytical or Python-based design
    double Kp = 1.1;
    double Ki = 11.0;
    double Kd = 0.02;

    // Frequency grid (rad/s)
    std::vector<double> omega = {1.0, 5.0, 10.0, 20.0, 40.0, 80.0};

    for (double w : omega) {
        complex<double> jw(0.0, w);

        // Plant: G(s) = K / (T s + 1)
        complex<double> G = K / (T * jw + 1.0);

        // PID: C(s) = Kp + Ki/s + Kd s, evaluated at s = j w
        complex<double> C = Kp + Ki / jw + Kd * jw;

        complex<double> L = C * G;

        double mag = std::abs(L);
        double phase = std::arg(L); // radians

        cout << "w = " << w
             << " |L(jw)| = " << mag
             << " phase(deg) = " << phase * 180.0 / M_PI
             << endl;
    }

    return 0;
}

/*
In a ROS-based robotic controller, the tuned gains Kp, Ki, Kd can be
used directly as:

  control_toolbox::Pid pid;
  pid.initPid(Kp, Ki, Kd, i_max, i_min);

The frequency-domain analysis above validates bandwidth and margins
before deploying the gains to the real robot.
*/
