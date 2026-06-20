// UART with termios (Linux)
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main() {
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    termios tio{};
    tcgetattr(fd, &tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;       // 8 data bits
    tio.c_cflag &= ~PARENB;   // no parity
    tio.c_cflag &= ~CSTOPB;   // 1 stop bit
    tcsetattr(fd, TCSANOW, &tio);

    unsigned char tx[5] = {0xAA, 0, 0, 0, 0};
    float v = 0.8f;
    std::memcpy(&tx[1], &v, 4);
    write(fd, tx, 5);

    unsigned char rx[5];
    int n = read(fd, rx, 5);
    if (n == 5 && rx[0] == 0x55) {
        float echo;
        std::memcpy(&echo, &rx[1], 4);
        std::cout << "Echo: " << echo << std::endl;
    }
    close(fd);
    return 0;
}
