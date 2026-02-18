#pragma once

#include "hydrolib_bus_datalink_deserializer.hpp"
#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_bus_datalink_serializer.hpp"
#include "hydrolib_stream_traits.hpp"
#include <cstring>
#include <type_traits>

namespace hydrolib::bus::datalink {

// ──────────────────────────────────────────────────────────────────────────────
// Traits access
// ──────────────────────────────────────────────────────────────────────────────

template <typename T>
inline constexpr bool is_byte_full_stream_v =
    hydrolib::concepts::stream::is_byte_full_stream_v<T>;

// ──────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ──────────────────────────────────────────────────────────────────────────────

template <typename RxTxStream, int MATES_COUNT = 3>
class StreamManager;

template <typename RxTxStream, int MATES_COUNT = 3>
class Stream;

// ──────────────────────────────────────────────────────────────────────────────
// StreamManager
// ──────────────────────────────────────────────────────────────────────────────

template <typename RxTxStream, int MATES_COUNT>
class StreamManager
{
    friend class Stream<RxTxStream, MATES_COUNT>;

private:
    using SerializerType   = Serializer<RxTxStream>;
    using DeserializerType = Deserializer<RxTxStream>;

public:
    constexpr StreamManager(AddressType self_address, RxTxStream &stream);

    void Process();

private:
    RxTxStream &stream_;
    const AddressType self_address_;

    DeserializerType deserializer_;

    Stream<RxTxStream, MATES_COUNT> *streams_[MATES_COUNT] = {nullptr};
    int streams_count_ = 0;
};

template <typename RxTxStream, int MATES_COUNT>
constexpr StreamManager<RxTxStream, MATES_COUNT>::StreamManager(
    AddressType self_address, RxTxStream &stream)
    : stream_(stream),
      self_address_(self_address),
      deserializer_(self_address, stream)
{
    static_assert(is_byte_full_stream_v<RxTxStream>,
                  "RxTxStream must provide int read(void*, unsigned) and int write(const void*, unsigned)");
}

template <typename RxTxStream, int MATES_COUNT>
void StreamManager<RxTxStream, MATES_COUNT>::Process()
{
    ReturnCode result = deserializer_.Process();
    if (result == ReturnCode::OK)
    {
        AddressType message_source_address = deserializer_.GetSourceAddress();
        unsigned message_length = deserializer_.GetDataLength();
        const uint8_t *message_data = deserializer_.GetData();

        for (int i = 0; i < streams_count_; ++i)
        {
            if (streams_[i] && streams_[i]->mate_address_ == message_source_address)
            {
                std::memcpy(streams_[i]->buffer, message_data, message_length);
                streams_[i]->message_length_ = message_length;
                break;
            }
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Stream
// ──────────────────────────────────────────────────────────────────────────────

template <typename RxTxStream, int MATES_COUNT>
class Stream
{
    friend class StreamManager<RxTxStream, MATES_COUNT>;

public:
    constexpr Stream(StreamManager<RxTxStream, MATES_COUNT> &stream_manager,
                     AddressType mate_address);

    int Read(void *dest, unsigned length);
    int Write(const void *dest, unsigned length);

private:
    StreamManager<RxTxStream, MATES_COUNT> &stream_manager_;
    const AddressType mate_address_;

    uint8_t buffer[kMaxMessageLength] = {};  // TODO: consider real queue
    unsigned message_length_ = 0;
};

template <typename RxTxStream, int MATES_COUNT>
constexpr Stream<RxTxStream, MATES_COUNT>::Stream(
    StreamManager<RxTxStream, MATES_COUNT> &stream_manager,
    AddressType mate_address)
    : stream_manager_(stream_manager),
      mate_address_(mate_address)
{
    stream_manager_.streams_[stream_manager_.streams_count_] = this;
    ++stream_manager_.streams_count_;
}

template <typename RxTxStream, int MATES_COUNT>
int Stream<RxTxStream, MATES_COUNT>::Read(void *dest, unsigned length)
{
    unsigned copy_len = (length > message_length_) ? message_length_ : length;
    std::memcpy(dest, buffer, copy_len);
    return static_cast<int>(copy_len);
}

template <typename RxTxStream, int MATES_COUNT>
int Stream<RxTxStream, MATES_COUNT>::Write(const void *dest, unsigned length)
{
    typename StreamManager<RxTxStream, MATES_COUNT>::SerializerType serializer(
        stream_manager_.self_address_, stream_manager_.stream_
    );
    ReturnCode result = serializer.Process(mate_address_, dest, length);
    return (result == ReturnCode::OK) ? static_cast<int>(length) : 0;
}

// ──────────────────────────────────────────────────────────────────────────────
// Free function wrappers
// ──────────────────────────────────────────────────────────────────────────────

template <typename RxTxStream, int MATES_COUNT = 3>
int read(Stream<RxTxStream, MATES_COUNT> &stream, void *dest, unsigned length)
{
    return stream.Read(dest, length);
}

template <typename RxTxStream, int MATES_COUNT = 3>
int write(Stream<RxTxStream, MATES_COUNT> &stream, const void *dest, unsigned length)
{
    return stream.Write(dest, length);
}

} // namespace hydrolib::bus::datalink
