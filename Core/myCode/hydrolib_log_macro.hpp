#pragma once

#include "hydrolib_logger.hpp"

#define LOG(logger_, level, message, ...)                           \
  hydrolib::logger::LoggingCase(logger_ __VA_OPT__(, ) __VA_ARGS__) \
      .WriteLog(level, message __VA_OPT__(, ) __VA_ARGS__)

#define LOG_DEBUG(logger_, message, ...) \
  LOG(logger_, hydrolib::logger::LogLevel::Debug, message, __VA_ARGS__)
#define LOG_INFO(logger_, message, ...) \
  LOG(logger_, hydrolib::logger::LogLevel::Info, message, __VA_ARGS__)
#define LOG_WARNING(logger_, message, ...) \
  LOG(logger_, hydrolib::logger::LogLevel::Warning, message, __VA_ARGS__)
#define LOG_ERROR(logger_, message, ...) \
  LOG(logger_, hydrolib::logger::LogLevel::Error, message, __VA_ARGS__)
#define LOG_CRITICAL(logger_, message, ...) \
  LOG(logger_, hydrolib::logger::LogLevel::Critical, message, __VA_ARGS__)
