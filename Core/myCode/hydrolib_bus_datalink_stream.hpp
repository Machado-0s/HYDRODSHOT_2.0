#pragma once

#include <algorithm>
#include <array>
#include <cstring>

#include "hydrolib_bus_datalink_deserializer.hpp"
#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_bus_datalink_serializer.hpp"
#include "hydrolib_log_macro.hpp"
#include "hydrolib_ring_queue.hpp"

namespace hydrolib::bus::datalink {
template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT = 3>
class Stream;

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT = 3>
class StreamManager {
  friend class Stream<RxTxStream, Logger, MATES_COUNT>;

 private:
  using SerializerType = Serializer<RxTxStream, Logger>;
  using DeserializerType = Deserializer<RxTxStream, Logger>;

 public:
  constexpr StreamManager(AddressType self_address, RxTxStream &stream,
                          Logger &logger);
  StreamManager(const StreamManager &) = delete;
  StreamManager(StreamManager &&) = delete;
  StreamManager &operator=(const StreamManager &) = delete;
  StreamManager &operator=(StreamManager &&) = delete;
  ~StreamManager() = default;

  void Process();
  [[nodiscard]] int GetLostBytes() const;

 private:
  RxTxStream &stream_;
  Logger &logger_;

  const AddressType self_address_;

  DeserializerType deserializer_;

  std::array<Stream<RxTxStream, Logger, MATES_COUNT> *, MATES_COUNT> streams_ =
      {nullptr};
  int streams_count_ = 0;
};

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
class Stream {
  friend class StreamManager<RxTxStream, Logger, MATES_COUNT>;

 public:
  constexpr Stream(
      StreamManager<RxTxStream, Logger, MATES_COUNT> &stream_manager,
      AddressType mate_address);
  Stream(const Stream &) = delete;
  Stream(Stream &&) = delete;
  Stream &operator=(const Stream &) = delete;
  Stream &operator=(Stream &&) = delete;
  ~Stream() = default;

  int Read(void *dest, int length);
  int Write(const void *dest, int length);

 private:
  StreamManager<RxTxStream, Logger, MATES_COUNT> &stream_manager_;
  const AddressType mate_address_;

  ring_queue::RingQueue<kMaxMessageLength> buffer_;
};

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
constexpr StreamManager<RxTxStream, Logger, MATES_COUNT>::StreamManager(
    AddressType self_address, RxTxStream &stream, Logger &logger)
    : stream_(stream),
      logger_(logger),
      self_address_(self_address),
      deserializer_(self_address, stream, logger) {}

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
void StreamManager<RxTxStream, Logger, MATES_COUNT>::Process() {
  ReturnCode result = deserializer_.Process();
  if (result == ReturnCode::OK) {
    AddressType message_source_address = deserializer_.GetSourceAddress();
    unsigned message_length = deserializer_.GetDataLength();
    const uint8_t *message_data = deserializer_.GetData();

    for (int i = 0; i < streams_count_; i++) {
      if (streams_[i]->mate_address_ == message_source_address) {
        auto push_result =
            streams_[i]->buffer_.Push(message_data, message_length);
        if (push_result != ReturnCode::OK) {
          LOG_ERROR(logger_, "Stream overflow");
        }
        break;
      }
    }
  }
}

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
int StreamManager<RxTxStream, Logger, MATES_COUNT>::GetLostBytes() const {
  return deserializer_.GetLostBytes();
}

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
constexpr Stream<RxTxStream, Logger, MATES_COUNT>::Stream(
    StreamManager<RxTxStream, Logger, MATES_COUNT> &stream_manager,
    AddressType mate_address)
    : stream_manager_(stream_manager), mate_address_(mate_address) {
  stream_manager.streams_[stream_manager.streams_count_] = this;
  stream_manager.streams_count_++;
}

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
int Stream<RxTxStream, Logger, MATES_COUNT>::Read(void *dest, int length) {
  int current_length = buffer_.GetLength();
  length = std::min<int>(length, current_length);
  buffer_.Pull(dest, length);
  return length;
}

template <concepts::stream::ByteFullStreamConcept RxTxStream, typename Logger,
          int MATES_COUNT>
int Stream<RxTxStream, Logger, MATES_COUNT>::Write(const void *dest,
                                                   int length) {
  typename StreamManager<RxTxStream, Logger, MATES_COUNT>::SerializerType
      serializer(stream_manager_.self_address_, stream_manager_.stream_,
                 stream_manager_.logger_);
  ReturnCode result = serializer.Process(mate_address_, dest, length);

  if (result == ReturnCode::OK) {
    return length;
  }

  return 0;
}

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          typename Logger, int MATES_COUNT = 3>
int read(typename hydrolib::bus::datalink::Stream<RxTxStream, Logger,
                                                  MATES_COUNT> &stream,
         void *dest, unsigned length);

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          typename Logger, int MATES_COUNT = 3>
int write(typename hydrolib::bus::datalink::Stream<RxTxStream, Logger,
                                                   MATES_COUNT> &stream,
          const void *dest, unsigned length);

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          typename Logger, int MATES_COUNT>
int read(typename hydrolib::bus::datalink::Stream<RxTxStream, Logger,
                                                  MATES_COUNT> &stream,
         void *dest, unsigned length) {
  return stream.Read(dest, length);
}

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          typename Logger, int MATES_COUNT>
int write(typename hydrolib::bus::datalink::Stream<RxTxStream, Logger,
                                                   MATES_COUNT> &stream,
          const void *dest, unsigned length) {
  return stream.Write(dest, length);
}

}  // namespace hydrolib::bus::datalink
