#ifndef SEEDLING_CHASSIS__SERIAL_PORT_HPP_
#define SEEDLING_CHASSIS__SERIAL_PORT_HPP_

#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace seedling_chassis
{

/// 极简 POSIX 串口封装（非阻塞读 + 阻塞写），只支持 Linux。
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  bool open(const std::string & device, int baudrate);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  /// 读取当前可用的字节，非阻塞；返回实际读到的字节数
  ssize_t readAvailable(std::vector<uint8_t> & buffer, size_t max_len);

  /// 阻塞写满整个缓冲区
  bool writeAll(const std::vector<uint8_t> & data);

private:
  int fd_ = -1;
};

}  // namespace seedling_chassis

#endif  // SEEDLING_CHASSIS__SERIAL_PORT_HPP_
