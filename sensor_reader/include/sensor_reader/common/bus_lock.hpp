#pragma once

#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace sensor_reader
{

class BusLock
{
public:
  explicit BusLock(const std::string & lock_file)
  {
    fd_ = ::open(lock_file.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open lock file: " + lock_file);
    }

    if (::flock(fd_, LOCK_EX) != 0) {
      ::close(fd_);
      throw std::runtime_error("Failed to lock file: " + lock_file);
    }
  }

  BusLock(const BusLock &) = delete;
  BusLock & operator=(const BusLock &) = delete;

  ~BusLock()
  {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

private:
  int fd_{-1};
};

}  // namespace sensor_reader
