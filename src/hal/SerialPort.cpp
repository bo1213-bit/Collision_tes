#include "SerialPort.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace hal {

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& port, int baud)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t speed = B115200;
    switch (baud) {
    case 9600:    speed = B9600;    break;
    case 19200:   speed = B19200;   break;
    case 38400:   speed = B38400;   break;
    case 57600:   speed = B57600;   break;
    case 115200:  speed = B115200;  break;
    case 230400:  speed = B230400;  break;
    case 460800:  speed = B460800;  break;
    case 921600:  speed = B921600;  break;
    default:      speed = B115200;  break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::is_open() const
{
    return fd_ >= 0;
}

int SerialPort::write(const uint8_t* data, int len)
{
    if (fd_ < 0) return -1;
    return ::write(fd_, data, len);
}

int SerialPort::read(uint8_t* buf, int len, int timeout_ms)
{
    if (fd_ < 0) return -1;

    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd_, &set);

    struct timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &set, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return ret; // 0 = timeout, -1 = error
    }

    int total = 0;
    while (total < len) {
        int n = ::read(fd_, buf + total, len - total);
        if (n > 0) {
            total += n;
        } else if (n == 0) {
            break;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            return -1;
        }
    }
    return total;
}

} // namespace hal
