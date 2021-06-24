#include "logger.h"

#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace slam {

static std::atomic<ELogLevel> GlobalLogLevel = ELogLevel::kDebug;

void SetLogLevel(ELogLevel log_level) {
  GlobalLogLevel.store(log_level, std::memory_order_release);
}

namespace util {


static constexpr const char* LogLevel2Str(ELogLevel log_level) {
  constexpr std::array<const char*, 6> map = {
    "ZERO ",
    "INFO ",
    "DEBUG",
    "WARN ",
    "ERROR",
    "FATAL",
  };

  return map[static_cast<uint8_t>(log_level)];
}


LogStream::LogStream(ELogLevel log_level,
                     const std::string& filename,
                     const std::string& func,
                     int line)
    : write_(log_level >= GlobalLogLevel.load(std::memory_order_release))
{
  if (write_) {
    message_ << '[' << LogLevel2Str(log_level) << "| "
             << filename << ':' << line << " | " << func << " ] ";
  }
}

LogStream::~LogStream() {
  if (write_) {
    std::string message_str = message_.str();
    printf("%s\n", message_str.c_str());
  }
}

}  // namespace util
}  // namespace slam

