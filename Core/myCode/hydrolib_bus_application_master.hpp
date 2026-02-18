#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include "hydrolib_bus_application_commands.hpp"
#include "hydrolib_return_codes.hpp"


#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_bus_datalink_message.hpp"

namespace hydrolib::bus::application {



template <typename T, typename = void>
struct has_read : std::false_type {};

template <typename T>
struct has_read<T, std::void_t<decltype(
    read(std::declval<T&>(), std::declval<void*>(), std::declval<unsigned>())
)>> : std::is_convertible<
    decltype(read(std::declval<T&>(), nullptr, 0u)),
    int
> {};


template <typename T, typename = void>
struct has_write : std::false_type {};

template <typename T>
struct has_write<T, std::void_t<decltype(
    write(std::declval<T&>(), std::declval<const void*>(), std::declval<unsigned>())
)>> : std::is_convertible<
    decltype(write(std::declval<T&>(), nullptr, 0u)),
    int
> {};


template <typename T>
inline constexpr bool is_byte_full_stream_v =
    has_read<T>::value && has_write<T>::value;


template <typename TxRxStream>
class Master
{
    static_assert(is_byte_full_stream_v<TxRxStream>,
        "TxRxStream must provide: int read(void*, unsigned) and int write(const void*, unsigned)");

public:
    constexpr Master(TxRxStream &stream);

    bool Process();
    void Read(void *data, unsigned address, unsigned length);
    void Write(const void *data, unsigned address, unsigned length);

private:
    TxRxStream &stream_;

    void *requested_data_ = nullptr;
    int requested_length_ = 0;

    uint8_t rx_buffer_[kMaxMessageLength] = {};
    uint8_t tx_buffer_[kMaxMessageLength] = {};
};


template <typename TxRxStream>
constexpr Master<TxRxStream>::Master(TxRxStream &stream)
    : stream_(stream)
{
    // buffers already zero-initialized via = {}
}


template <typename TxRxStream>
bool Master<TxRxStream>::Process()
{
    if (!requested_data_)
    {
        return false;
    }

    int read_length = read(stream_, rx_buffer_, kMaxMessageLength);
    if (read_length <= 0)
    {
        return false;
    }

    Command command = *reinterpret_cast<Command *>(rx_buffer_);

    switch (command)
    {
    case Command::RESPONSE:
        if (requested_length_ + static_cast<int>(sizeof(Command)) != read_length)
        {
            return false;
        }

        memcpy(requested_data_, rx_buffer_ + sizeof(Command), requested_length_);
        requested_data_ = nullptr;
        return true;

    case Command::ERROR:
    case Command::READ:
    case Command::WRITE:
    default:
        // Logging removed — silently fail
        return false;
    }
}


template <typename TxRxStream>
void Master<TxRxStream>::Read(void *data, unsigned address, unsigned length)
{
    requested_data_ = data;
    requested_length_ = length;

    // Using MemoryAccessHeader (assumed alias or struct from commands.hpp)
    MemoryAccessHeader *header = reinterpret_cast<MemoryAccessHeader *>(tx_buffer_);
    header->command = Command::READ;
    header->info.address = address;
    header->info.length = length;

    write(stream_, tx_buffer_, sizeof(MemoryAccessHeader));
}


template <typename TxRxStream>
void Master<TxRxStream>::Write(const void *data, unsigned address, unsigned length)
{
    MemoryAccessHeader *header = reinterpret_cast<MemoryAccessHeader *>(tx_buffer_);
    header->command = Command::WRITE;
    header->info.address = address;
    header->info.length = length;

    memcpy(tx_buffer_ + sizeof(MemoryAccessHeader), data, length);

    write(stream_, tx_buffer_, sizeof(MemoryAccessHeader) + length);
}

} // namespace hydrolib::bus::application
