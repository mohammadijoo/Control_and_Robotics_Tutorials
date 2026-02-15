#include <iostream>
#include <functional>
#include <unordered_map>
#include <vector>

// L1: minimal bus
class Bus {
public:
  using Callback = std::function<void(double)>;
  void publish(const std::string& topic, double msg) {
    for (auto& cb : subs_[topic]) cb(msg);
  }
  void subscribe(const std::string& topic, Callback cb) {
    subs_[topic].push_back(cb);
  }
private:
  std::unordered_map<std::string, std::vector<Callback>> subs_;
};

// L0: driver interface
class EncoderDriver {
public:
  EncoderDriver(int cpr=4096): cpr_(cpr), counts_(0) {}
  int readCounts() const { return counts_; }
  void writePWM(double pwm){ counts_ += static_cast<int>(0.1*pwm); }
  int cpr() const { return cpr_; }
private:
  int cpr_, counts_;
};

// L2: controller
class PositionController {
public:
  PositionController(Bus& bus, double kp=0.2): bus_(bus), kp_(kp), ref_(0.0) {
    bus_.subscribe("ref", [&](double r){ ref_=r; });
    bus_.subscribe("pos", [&](double p){ onPos(p); });
  }
  void onPos(double pos){
    double u = kp_*(ref_ - pos);
    bus_.publish("cmd_pwm", u);
  }
private:
  Bus& bus_; double kp_, ref_;
};

int main(){
  EncoderDriver drv;
  Bus bus;
  PositionController ctrl(bus);
  bus.subscribe("cmd_pwm", [&](double u){ drv.writePWM(u); });

  double t=0.0, dt=0.01;
  for(int k=0;k<400;k++){
    double ref = (t>1.0)?10.0:0.0;
    bus.publish("ref", ref);

    int counts = drv.readCounts();
    double pos = counts / double(drv.cpr()) * 2.0 * 3.14159;
    bus.publish("pos", pos);

    t += dt;
  }
}
      
