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


constexpr std::array<const char*, 6> LogLevel2Str = {
  "ZERO ",
  "INFO ",
  "DEBUG",
  "WARN ",
  "ERROR",
  "FATAL",
};


static void LogPrefix(ELogLevel log_level,
          const std::string& filename, const std::string& func, int line) {
  printf("[%s| file=%s:%d | func=%s ] ",
         LogLevel2Str[static_cast<uint8_t>(log_level)],
         filename.c_str(), line,
         func.c_str());
}


void LogF(ELogLevel log_level,
          const std::string& filename, const std::string& func, int line,
          const char* format, ...) {
  if (static_cast<uint8_t>(log_level) < static_cast<uint8_t>(
        GlobalLogLevel.load(std::memory_order_release))) {
    return;
  }
  std::va_list arg;
  va_start(arg, format);
  LogPrefix(log_level, filename, func, line);
  vprintf(format, arg);
  // throw std::runtime_error("vprintf error");
  // }
  va_end(arg);
}

}  // namespace util
}  // namespace slam

