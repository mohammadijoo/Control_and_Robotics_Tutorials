#include <iostream>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    const char* server_ip = "192.168.1.10";
    const int port = 9000;
    const int N = 100;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { std::cerr << "socket error\n"; return 1; }

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, server_ip, &server.sin_addr);

    char msg[32] = "ping";
    char buf[64];

    double rtt_sum = 0.0;
    for (int i = 0; i < N; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        sendto(sock, msg, strlen(msg), 0,
               (sockaddr*)&server, sizeof(server));

        socklen_t len = sizeof(server);
        recvfrom(sock, buf, sizeof(buf), 0,
                 (sockaddr*)&server, &len);
        auto t1 = std::chrono::steady_clock::now();

        double rtt = std::chrono::duration<double>(t1 - t0).count();
        rtt_sum += rtt;
    }

    std::cout << "Avg RTT: " << (rtt_sum / N) << " s\n";
    close(sock);
    return 0;
}
