#pragma once

#include <unistd.h>

#include <chrono>
#include <cstring>

#include "hydrolib_bus_application_commands.hpp"
#include "hydrolib_log_macro.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::bus::application {
using namespace std::literals::chrono_literals;

template <concepts::stream::ByteFullStreamConcept TxRxStream, typename Logger>
class Master {
 public:
  static constexpr auto kRequestTimeout = 1s;

  constexpr Master(TxRxStream &stream, Logger &logger);
  Master(const Master &) = delete;
  Master(Master &&) = delete;
  Master &operator=(const Master &) = delete;
  Master &operator=(Master &&) = delete;
  ~Master() = default;

  hydrolib::ReturnCode Process();
  void Read(void *data, int address, int length);
  void Write(const void *data, int address, int length);

 private:
  TxRxStream &stream_;
  Logger &logger_;

  void *requested_data_ = nullptr;
  int requested_length_ = 0;

  ResponseMessageBuffer rx_buffer_{};
  MemoryAccessMessageBuffer tx_buffer_{};

  std::chrono::steady_clock::time_point last_request_time_;
};

template <concepts::stream::ByteFullStreamConcept TxRxStream, typename Logger>
constexpr Master<TxRxStream, Logger>::Master(TxRxStream &stream, Logger &logger)
    : stream_(stream), logger_(logger) {}

template <concepts::stream::ByteFullStreamConcept TxRxStream, typename Logger>
hydrolib::ReturnCode Master<TxRxStream, Logger>::Process() {
  if (requested_data_ == nullptr) {
    return hydrolib::ReturnCode::FAIL;
  }

  if (std::chrono::steady_clock::now() - last_request_time_ > kRequestTimeout) {
    LOG_ERROR(logger_, "Request timeout");
    write(stream_, &tx_buffer_, sizeof(MemoryAccessHeader));
    last_request_time_ = std::chrono::steady_clock::now();
    return hydrolib::ReturnCode::TIMEOUT;
  }

  int read_length = read(stream_, &rx_buffer_, kMaxMessageLength);
  if (read_length == 0) {
    return hydrolib::ReturnCode::NO_DATA;
  }

  switch (rx_buffer_.command) {
    case Command::RESPONSE:
      if (requested_length_ + static_cast<int>(sizeof(Command)) !=
          read_length) {
        return hydrolib::ReturnCode::ERROR;
      }
      memcpy(requested_data_, static_cast<void *>(rx_buffer_.data),
             requested_length_);
      requested_data_ = nullptr;
      return hydrolib::ReturnCode::OK;
    case Command::ERROR:
    case Command::READ:
    case Command::WRITE:
    default:
      LOG_WARNING(logger_, "Wrong command");
      return hydrolib::ReturnCode::ERROR;
  }
}

template <concepts::stream::ByteFullStreamConcept TxRxStream, typename Logger>
void Master<TxRxStream, Logger>::Read(void *data, int address, int length) {
  requested_data_ = data;
  requested_length_ = length;

  tx_buffer_.header.command = Command::READ;
  tx_buffer_.header.info.address = address;
  tx_buffer_.header.info.length = length;

  write(stream_, &tx_buffer_, sizeof(MemoryAccessHeader));
  last_request_time_ = std::chrono::steady_clock::now();
}

template <concepts::stream::ByteFullStreamConcept TxRxStream, typename Logger>
void Master<TxRxStream, Logger>::Write(const void *data, int address,
                                       int length) {
  tx_buffer_.header.command = Command::WRITE;
  tx_buffer_.header.info.address = address;
  tx_buffer_.header.info.length = length;

  memcpy(static_cast<void *>(tx_buffer_.data), data, length);

  write(stream_, &tx_buffer_, sizeof(MemoryAccessHeader) + length);
}

}  // namespace hydrolib::bus::application
