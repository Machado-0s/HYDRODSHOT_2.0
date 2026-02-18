#pragma once

#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_crc.hpp"
#include "hydrolib_return_codes.hpp"
#include <cstring>

namespace hydrolib::bus::datalink {

template <typename TxStream>
class Serializer
{
public:
    constexpr Serializer(AddressType self_address, TxStream &tx_stream);

    ReturnCode Process(AddressType dest_address, const void *data,
                       unsigned data_length);

private:
    const AddressType address_;
    TxStream &tx_stream_;

    uint8_t current_message_[kMaxMessageLength] = {};
    MessageHeader *current_header_;
};

template <typename TxStream>
constexpr Serializer<TxStream>::Serializer(
    AddressType address, TxStream &tx_stream)
    : address_(address),
      tx_stream_(tx_stream),
      current_header_(reinterpret_cast<MessageHeader *>(current_message_))
{
    current_header_->magic_byte = kMagicByte;
}

template <typename TxStream>
ReturnCode Serializer<TxStream>::Process(AddressType dest_address,
                                         const void *data,
                                         unsigned data_length)
{
    current_header_->dest_address = dest_address;
    current_header_->src_address  = address_;
    current_header_->length = static_cast<uint8_t>(data_length + kCRCLength);

    memcpy(current_message_ + sizeof(MessageHeader), data, data_length);

    uint8_t crc_val = crc::CountCRC8(current_message_, sizeof(MessageHeader) + data_length);
    current_message_[sizeof(MessageHeader) + data_length] = crc_val;

    unsigned total_length = sizeof(MessageHeader) + data_length + kCRCLength;
    int res = write(tx_stream_, current_message_, total_length);

    if (res < 0) return ReturnCode::ERROR;
    if ((unsigned)res != total_length) return ReturnCode::OVERFLOW;
    return ReturnCode::OK;
}

} // namespace hydrolib::bus::datalink
