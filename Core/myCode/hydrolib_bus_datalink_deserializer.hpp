#pragma once

#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_crc.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_traits.hpp"
#include "hal_uart_adapter.hpp"

#include <type_traits>
#include <utility>
#include <cstring>

namespace hydrolib::bus::datalink {

template <typename RxStream>
class Deserializer
{
public:
    constexpr Deserializer(AddressType address, RxStream &rx_stream);

    ReturnCode Process();

    AddressType GetSourceAddress() const;
    const uint8_t *GetData();
    unsigned GetDataLength() const;

private:
    ReturnCode FindHeader_();
    ReturnCode ParseHeader_();
    bool CheckCRC_();

private:
    const AddressType self_address_;
    RxStream &rx_stream_;

    unsigned current_processed_length_ = 0;
    uint8_t *current_rx_buffer_;
    uint8_t *next_rx_buffer_;
    bool message_ready_ = false;

    uint8_t first_rx_buffer_[kMaxMessageLength] = {};
    uint8_t second_rx_buffer_[kMaxMessageLength] = {};

    MessageHeader *current_header_;
};

template <typename RxStream>
constexpr Deserializer<RxStream>::Deserializer(
    AddressType address, RxStream &rx_stream)
    : self_address_(address),
      rx_stream_(rx_stream),
      current_rx_buffer_(first_rx_buffer_),
      next_rx_buffer_(second_rx_buffer_),
      current_header_(reinterpret_cast<MessageHeader *>(current_rx_buffer_))
{
}

template <typename RxStream>
ReturnCode Deserializer<RxStream>::Process()
{
    while (true)
    {
        if (current_processed_length_ == 0)
        {
            ReturnCode res = FindHeader_();
            if (res != ReturnCode::OK) return res;
        }

        if (current_processed_length_ < sizeof(MessageHeader))
        {
            ReturnCode res = ParseHeader_();
            if (res == ReturnCode::FAIL) continue;
            if (res != ReturnCode::OK) return res;
        }

        // Total expected = Header + Python's length (Payload + CRC)
        unsigned total_expected = sizeof(MessageHeader) + current_header_->length;

        int res = read(
            rx_stream_,
            current_rx_buffer_ + current_processed_length_,
            total_expected - current_processed_length_
        );

        if (res > 0) current_processed_length_ += res;

        if (current_processed_length_ < total_expected)
        {
            return ReturnCode::NO_DATA;
        }

        if (CheckCRC_())
        {
            uint8_t *temp = current_rx_buffer_;
            current_rx_buffer_ = next_rx_buffer_;
            next_rx_buffer_ = temp;
            message_ready_ = true;
            current_processed_length_ = 0;
            return ReturnCode::OK;
        }

        current_processed_length_ = 0;
    }
}

// RESTORED: This was missing and caused your linker error
template <typename RxStream>
AddressType Deserializer<RxStream>::GetSourceAddress() const
{
    if (!message_ready_) return 0;
    MessageHeader *header = reinterpret_cast<MessageHeader *>(next_rx_buffer_);
    return header->src_address;
}

template <typename RxStream>
const uint8_t *Deserializer<RxStream>::GetData()
{
    if (!message_ready_) return nullptr;
    message_ready_ = false;
    return next_rx_buffer_ + sizeof(MessageHeader);
}

// RESTORED: This was missing and caused your linker error
template <typename RxStream>
unsigned Deserializer<RxStream>::GetDataLength() const
{
    if (!message_ready_) return 0;
    MessageHeader *header = reinterpret_cast<MessageHeader *>(next_rx_buffer_);
    return header->length - kCRCLength;
}

template <typename RxStream>
ReturnCode Deserializer<RxStream>::FindHeader_()
{
    int res = read(rx_stream_, current_rx_buffer_, sizeof(kMagicByte));
    if (res <= 0) return ReturnCode::NO_DATA;

    if (current_rx_buffer_[0] == kMagicByte)
    {
        current_processed_length_ = sizeof(kMagicByte);
        return ReturnCode::OK;
    }
    return ReturnCode::NO_DATA;
}

template <typename RxStream>
ReturnCode Deserializer<RxStream>::ParseHeader_()
{
    int res = read(
        rx_stream_,
        current_rx_buffer_ + current_processed_length_,
        sizeof(MessageHeader) - current_processed_length_
    );

    if (res > 0) current_processed_length_ += res;

    if (current_processed_length_ != sizeof(MessageHeader))
        return ReturnCode::NO_DATA;

    if (current_header_->dest_address != self_address_)
    {
        current_processed_length_ = 0;
        return ReturnCode::FAIL;
    }
    return ReturnCode::OK;
}

template <typename RxStream>
bool Deserializer<RxStream>::CheckCRC_()
{

    uint8_t target_crc = crc::CountCRC8(
        current_rx_buffer_,
        sizeof(MessageHeader) + current_header_->length - kCRCLength
    );

    uint8_t received_crc = current_rx_buffer_[sizeof(MessageHeader) + current_header_->length - kCRCLength];

    return target_crc == received_crc;
}

} // namespace hydrolib::bus::datalink
