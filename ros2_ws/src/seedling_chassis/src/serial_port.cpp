#include "seedling_chassis/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace seedling_chassis
{

namespace
{

speed_t toBaud(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string & device, int baudrate)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  ::cfmakeraw(&tty);
  ::cfsetispeed(&tty, toBaud(baudrate));
  ::cfsetospeed(&tty, toBaud(baudrate));

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // 关掉软件/硬件流控
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag &= ~CRTSCTS;

  // 读：立即返回；写：累计到 10 ms 再发
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  ::tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::readAvailable(std::vector<uint8_t> & buffer, size_t max_len)
{
  if (fd_ < 0) {
    return -1;
  }
  buffer.resize(max_len);
  const ssize_t n = ::read(fd_, buffer.data(), max_len);
  if (n < 0) {
    buffer.clear();
    return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
  }
  buffer.resize(static_cast<size_t>(n));
  return n;
}

bool SerialPort::writeAll(const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return false;
  }
  size_t written = 0;
  while (written < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n < 0) {
      if (errno == EAGAIN) {
        ::usleep(1000);
        continue;
      }
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace seedling_chassis
