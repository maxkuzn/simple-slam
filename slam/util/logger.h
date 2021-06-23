#pragma once

#include <sstream>
#include <string>

namespace slam {

enum class ELogLevel : uint8_t {
  kInfo = 1,
  kDebug = 2,
  kWarn = 3,
  kError = 4,
  kFatal = 5,
};

void SetLogLevel(ELogLevel log_level);

namespace util {

class LogStream {
 public:
  LogStream(ELogLevel log_level,
            const std::string& filename, const std::string& func, int line);

  ~LogStream();

  template <typename T>
  LogStream& operator<<(const T& item) {
    message_ << item;
    return *this;
  }

 private:
  std::stringstream message_;
};

#ifndef __FILE_NAME__
#define __FILE_NAME__ __FILE__
#endif

#define Log(log_level) \
  ::slam::util::LogStream(log_level, __FILE_NAME__, __func__, __LINE__)

#define LogInfo()  Log(slam::ELogLevel::kInfo)
#define LogDebug() Log(slam::ELogLevel::kDebug)
#define LogWarn()  Log(slam::ELogLevel::kWarn)
#define LogError() Log(slam::ELogLevel::kError)
#define LogFatal() Log(slam::ELogLevel::kFatal)


}  // namespace util
}  // namespace slam

