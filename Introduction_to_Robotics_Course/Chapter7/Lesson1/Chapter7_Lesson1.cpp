#include <iostream>
#include <random>

class Sensor {
public:
    virtual double read(double state_p, double state_v) = 0;
    virtual ~Sensor() {}
};

class ProprioVelocitySensor : public Sensor {
    double bias, sigma;
    std::default_random_engine gen;
    std::normal_distribution<double> noise;
public:
    ProprioVelocitySensor(double b, double s)
      : bias(b), sigma(s), noise(0.0, s) {}
    double read(double p, double v) override {
        return v + bias + noise(gen);
    }
};

class ExteroPositionSensor : public Sensor {
    double sigma;
    std::default_random_engine gen;
    std::normal_distribution<double> noise;
public:
    ExteroPositionSensor(double s)
      : sigma(s), noise(0.0, s) {}
    double read(double p, double v) override {
        return p + noise(gen);
    }
};

int main() {
    ProprioVelocitySensor enc(0.05, 0.02);
    ExteroPositionSensor rng(0.05);

    double p = 0.0, v = 1.0, Ts = 0.1;
    double p_hat = 0.0;

    for(int k=0;k<200;k++){
        p += Ts*v;  // true motion

        double y_p = enc.read(p, v);
        double y_e = rng.read(p, v);

        p_hat += Ts*y_p; // drift-prone integration

        if(k % 50 == 0){
            std::cout << "k=" << k
                      << " true p=" << p
                      << " proprio p_hat=" << p_hat
                      << " extero y_e=" << y_e << "\n";
        }
    }
}
