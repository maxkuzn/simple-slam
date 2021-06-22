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

__attribute__ ((format (printf, 5, 6)))
void LogF(ELogLevel log_level,
          const std::string& filename, const std::string& func, int line,
          const char* format, ...);

class LogStream {
 public:
  LogStream(ELogLevel log_level,
            const std::string& filename, const std::string& func, int line)
    : log_level_(log_level)
    , filename_(filename)
    , func_(func)
    , line_(line)
  {
  }

  ~LogStream() {
    auto message_str = message_.str();
    LogF(log_level_, filename_, func_, line_, "%s", message_str.c_str());
  }

  template <typename T>
  LogStream& operator<<(const T& item) {
    message_ << item;
    return *this;
  }

 private:
  const ELogLevel log_level_;
  const std::string filename_;
  const std::string func_;
  const int line_;

  std::stringstream message_;
};


#define LogS(log_level) \
  LogStream(log_level, __FILE__, __FUNCTION__, __LINE__)

#define  InfoLogS LogS(slam::ELogLevel::kInfo)
#define DebugLogS LogS(slam::ELogLevel::kDebug)
#define  WarnLogS LogS(slam::ELogLevel::kWarn)
#define ErrorLogS LogS(slam::ELogLevel::kError)
#define FatalLogS LogS(slam::ELogLevel::kFatal)


}  // namespace util
}  // namespace slam

