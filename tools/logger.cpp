#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

spdlog::level::level_enum log_level_from_env()
{
  const char * raw_level = std::getenv("HFUT_LOG_LEVEL");
  if (raw_level == nullptr) return spdlog::level::info;

  std::string level(raw_level);
  std::transform(level.begin(), level.end(), level.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (level == "trace") return spdlog::level::trace;
  if (level == "debug") return spdlog::level::debug;
  if (level == "warn" || level == "warning") return spdlog::level::warn;
  if (level == "error") return spdlog::level::err;
  if (level == "off") return spdlog::level::off;
  return spdlog::level::info;
}

void set_logger()
{
  const auto level = log_level_from_env();
  std::filesystem::create_directories("logs");
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(level);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(level);

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(level);
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
