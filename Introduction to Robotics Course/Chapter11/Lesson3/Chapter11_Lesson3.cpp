// g++ pubsub.cpp -lzmq -std=c++17
#include <iostream>
#include <thread>
#include <chrono>
#include <zmq.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void publisher(double rate_hz){
    zmq::context_t ctx(1);
    zmq::socket_t pub(ctx, ZMQ_PUB);
    pub.bind("tcp://*:5556");
    int k=0;
    auto T = std::chrono::duration<double>(1.0/rate_hz);
    while(true){
        json msg = {
          {"seq",k},{"t_pub",(double)std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count()}
          };
        std::string out = "imu " + msg.dump();
        pub.send(zmq::buffer(out), zmq::send_flags::none);
        k++;
        std::this_thread::sleep_for(T);
    }
}

void subscriber(double mu_hz){
    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.connect("tcp://localhost:5556");
    sub.setsockopt(ZMQ_SUBSCRIBE, "imu", 3);
    auto T = std::chrono::duration<double>(1.0/mu_hz);
    while(true){
        zmq::message_t m;
        sub.recv(m, zmq::recv_flags::none);
        std::string s((char*)m.data(), m.size());
        auto pos = s.find(" ");
        std::string payload = s.substr(pos+1);
        auto msg = json::parse(payload);
        double now = (double)std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "seq=" << msg["seq"]
                  << " age=" << (now - (double)msg["t_pub"]) << "s\n";
        std::this_thread::sleep_for(T);
    }
}

int main(){
    std::thread(pub, publisher, 50.0).detach();
    subscriber(40.0);
}
      
