#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>  // std::declval, std::void_t

#include "hydrolib_bus_application_commands.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_bus_datalink_stream.hpp"   // for read/write functions
#include "hydrolib_bus_datalink_message.hpp"  // for kMaxMessageLength

namespace hydrolib::bus::application {


template <typename T, typename = void>
struct has_read_func : std::false_type {};

template <typename T>
struct has_read_func<T, std::void_t<decltype(
    read(std::declval<T&>(), std::declval<void*>(), std::declval<unsigned>())
)>> : std::is_convertible<
    decltype(read(std::declval<T&>(), nullptr, 0u)),
    int
> {};


template <typename T, typename = void>
struct has_write_func : std::false_type {};

template <typename T>
struct has_write_func<T, std::void_t<decltype(
    write(std::declval<T&>(), std::declval<const void*>(), std::declval<unsigned>())
)>> : std::is_convertible<
    decltype(write(std::declval<T&>(), nullptr, 0u)),
    int
> {};


template <typename T, typename = void>
struct has_memory_read : std::false_type {};

template <typename T>
struct has_memory_read<T, std::void_t<decltype(
    std::declval<T>().Read(nullptr, 0u, 0u)
)>> : std::is_same<
    decltype(std::declval<T>().Read(nullptr, 0u, 0u)),
    ReturnCode
> {};


template <typename T, typename = void>
struct has_memory_write : std::false_type {};

template <typename T>
struct has_memory_write<T, std::void_t<decltype(
    std::declval<T>().Write(nullptr, 0u, 0u)
)>> : std::is_same<
    decltype(std::declval<T>().Write(nullptr, 0u, 0u)),
    ReturnCode
> {};


template <typename T>
inline constexpr bool is_byte_stream_v =
    has_read_func<T>::value && has_write_func<T>::value;

template <typename T>
inline constexpr bool is_public_memory_v =
    has_memory_read<T>::value && has_memory_write<T>::value;

// ──────────────────────────────────────────────────────────────────────────────
// Slave — no logger, no concepts
// ──────────────────────────────────────────────────────────────────────────────

template <typename Memory, typename TxRxStream>
class Slave
{
    static_assert(is_public_memory_v<Memory>,
        "Memory must provide ReturnCode Read(void*, unsigned, unsigned) and ReturnCode Write(const void*, unsigned, unsigned)");

    static_assert(is_byte_stream_v<TxRxStream>,
        "TxRxStream must provide int read(void*, unsigned) and int write(const void*, unsigned)");

public:
    constexpr Slave(TxRxStream &stream, Memory &memory);
    // Note: logger removed from constructor

    void Process();

private:
    TxRxStream &stream_;
    Memory &memory_;

    uint8_t rx_buffer_[kMaxMessageLength] = {};
    uint8_t tx_buffer_[kMaxMessageLength] = {};
};


template <typename Memory, typename TxRxStream>
constexpr Slave<Memory, TxRxStream>::Slave(TxRxStream &stream, Memory &memory)
    : stream_(stream), memory_(memory)
{
    // buffers zero-initialized via = {}
}


template <typename Memory, typename TxRxStream>
void Slave<Memory, TxRxStream>::Process()
{
    int read_length = read(stream_, rx_buffer_, kMaxMessageLength);
    if (read_length <= 0)
    {
        return;
    }

    Command command = *reinterpret_cast<Command *>(rx_buffer_);

    switch (command)
    {
    case Command::READ:
    {
        MemoryAccessInfo *info =
            reinterpret_cast<MemoryAccessInfo *>(rx_buffer_ + sizeof(Command));

        ReturnCode res = memory_.Read(
            tx_buffer_ + sizeof(Command),
            info->address,
            info->length
        );

        if (res == ReturnCode::OK)
        {
            *reinterpret_cast<Command *>(tx_buffer_) = Command::RESPONSE;
            write(stream_, tx_buffer_, sizeof(Command) + info->length);
        }
        else
        {
            *reinterpret_cast<Command *>(tx_buffer_) = Command::ERROR;
            write(stream_, tx_buffer_, sizeof(Command));
        }
    }
    break;

    case Command::WRITE:
    {
        MemoryAccessInfo *info =
            reinterpret_cast<MemoryAccessInfo *>(rx_buffer_ + sizeof(Command));

        ReturnCode res = memory_.Write(
            rx_buffer_ + sizeof(Command) + sizeof(MemoryAccessInfo),
            info->address,
            info->length
        );

        if (res != ReturnCode::OK)
        {
            *reinterpret_cast<Command *>(tx_buffer_) = Command::ERROR;
            write(stream_, tx_buffer_, sizeof(Command));
        }
        // success case: no response sent (as in original)
    }
    break;

    case Command::ERROR:
    case Command::RESPONSE:
    default:
        *reinterpret_cast<Command *>(tx_buffer_) = Command::ERROR;
        write(stream_, tx_buffer_, sizeof(Command));
        break;
    }
}

} // namespace hydrolib::bus::application