#ifndef _LUXCORE_SINKS_H
#define _LUXCORE_SINKS_H

#include <mutex>

#include "spdlog/sinks/rotating_file_sink.h"

namespace spdlog {
namespace sinks {
template<typename Mutex>
class luxcore_callback_sink final : public base_sink<Mutex> {
 public:
  explicit luxcore_callback_sink(void (*handler)(const char *)) : logHandler(handler) {}

 protected:
  void sink_it_(const details::log_msg &msg) override
  {
    if (logHandler) {
      memory_buf_t formatted;
      base_sink<Mutex>::formatter_->format(msg, formatted);
      formatted.push_back('\0');
      logHandler(formatted.data());
    }
  }

  void flush_() override {}

 private:
  void (*logHandler)(const char *msg);
};

using luxcore_callback_sink_mt = luxcore_callback_sink<std::mutex>;
using luxcore_callback_sink_st = luxcore_callback_sink<details::null_mutex>;

}  // namespace sinks

template<typename Factory = spdlog::synchronous_factory>
inline std::shared_ptr<logger> luxcore_callback_mt(const std::string &logger_name,
                                                   void (*handler)(const char *))
{
  return Factory::template create<sinks::luxcore_callback_sink_mt>(logger_name, handler);
}

template<typename Factory = spdlog::synchronous_factory>
inline std::shared_ptr<logger> luxcore_callback_st(const std::string &logger_name,
                                                   void (*handler)(const char *))
{
  return Factory::template create<sinks::luxcore_callback_sink_st>(logger_name, handler);
}
}  // namespace spdlog

#endif
