#pragma once

#include <cstdint>
#include <algorithm> 

#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_crc.hpp"
#include "hydrolib_log_macro.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::bus::datalink {

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
class Deserializer {
 public:
  constexpr Deserializer(AddressType address, RxStream &rx_stream,
                         Logger &logger);

  Deserializer(const Deserializer &) = delete;
  Deserializer(Deserializer &&) = delete;
  Deserializer &operator=(const Deserializer &) = delete;
  Deserializer &operator=(Deserializer &&) = delete;
  ~Deserializer() = default;

  ReturnCode Process();

  AddressType GetSourceAddress() const;
  const uint8_t *GetData();
  unsigned GetDataLength() const;
  int GetLostBytes() const;

  static ReturnCode COBSDecoding(uint8_t magic_byte, uint8_t *data,
                                 unsigned data_length);

 private:
  ReturnCode FindHeader_();
  ReturnCode ParseHeader_();
  bool CheckCRC_();

  const AddressType self_address_;

  RxStream &rx_stream_;
  Logger &logger_;

  MessageBuffer first_rx_buffer_{};
  MessageBuffer second_rx_buffer_{};

  unsigned current_processed_length_ = 0;
  MessageBuffer *current_rx_buffer_ = &first_rx_buffer_;
  MessageBuffer *next_rx_buffer_ = &second_rx_buffer_;
  bool message_ready_ = false;

  MessageHeader *current_header_ = &current_rx_buffer_->header;

  int lost_bytes_ = 0;
};

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
constexpr Deserializer<RxStream, Logger>::Deserializer(AddressType address,
                                                       RxStream &rx_stream,
                                                       Logger &logger)
    : self_address_(address), rx_stream_(rx_stream), logger_(logger) {}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
ReturnCode Deserializer<RxStream, Logger>::Process() {
    constexpr int kMaxBytesScannedPerCall = 64;

    int bytes_scanned_this_call = 0;

    while (bytes_scanned_this_call < kMaxBytesScannedPerCall) {
        if (current_processed_length_ == 0) {
            ReturnCode res = FindHeader_();
            if (res == ReturnCode::NO_DATA) {
                break;
            }
            if (res != ReturnCode::OK) {
                return res;
            }
            bytes_scanned_this_call += 1;
        }

        if (current_processed_length_ < sizeof(MessageHeader)) {
            ReturnCode res = ParseHeader_();
            if (res == ReturnCode::FAIL) {
                current_processed_length_ = 0;
                bytes_scanned_this_call += sizeof(MessageHeader);
                continue;
            }
            if (res != ReturnCode::OK) {
                break;
            }
        }

        if (current_header_->length < sizeof(MessageHeader) + kCRCLength ||
            current_header_->length > sizeof(MessageBuffer)) {
            lost_bytes_ += current_processed_length_;
            current_processed_length_ = 0;
            continue;
        }

        int wanted = current_header_->length - current_processed_length_;
        int got = read(rx_stream_,
                       &current_rx_buffer_->data_and_crc[current_processed_length_ - sizeof(MessageHeader)],
                       wanted);

        if (got <= 0) {
            if (got < 0) return ReturnCode::ERROR;
            break;
        }

        current_processed_length_ += got;
        bytes_scanned_this_call += got;

        if (current_processed_length_ < current_header_->length) {
            break;
        }

        unsigned decode_len = current_header_->length - sizeof(MessageHeader) +
                              sizeof(MessageHeader::cobs_length);

        ReturnCode res = COBSDecoding(kMagicByte,
                                      &current_rx_buffer_->header.cobs_length,
                                      decode_len);

        if (res == ReturnCode::OK && CheckCRC_()) {
            // Success: swap double buffers
            auto *temp = current_rx_buffer_;
            current_rx_buffer_ = next_rx_buffer_;
            next_rx_buffer_ = temp;
            current_header_ = &current_rx_buffer_->header;
            message_ready_ = true;
            current_processed_length_ = 0;
            return ReturnCode::OK;
        }

        lost_bytes_ += current_processed_length_;
        current_processed_length_ = 0;
    }

    return message_ready_ ? ReturnCode::OK : ReturnCode::NO_DATA;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
AddressType Deserializer<RxStream, Logger>::GetSourceAddress() const {
    if (message_ready_) {
        return current_rx_buffer_->header.src_address;  // FIXED: current, not next
    }
    return 0;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
const uint8_t *Deserializer<RxStream, Logger>::GetData() {
    if (message_ready_) {
        message_ready_ = false;
        return static_cast<const uint8_t *>(current_rx_buffer_->data_and_crc);  // FIXED: current, not next
    }
    return nullptr;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
unsigned Deserializer<RxStream, Logger>::GetDataLength() const {
    if (message_ready_) {
        return current_rx_buffer_->header.length - sizeof(MessageHeader) - kCRCLength;  // FIXED: current, not next
    }
    return 0;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
int Deserializer<RxStream, Logger>::GetLostBytes() const {
    return lost_bytes_;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
ReturnCode Deserializer<RxStream, Logger>::COBSDecoding(uint8_t magic_byte,
                                                        uint8_t *data,
                                                        unsigned data_length) {
    if (data_length == 0 || data_length > kMaxDataLength + kCRCLength + 16) {
        return ReturnCode::ERROR;
    }

    unsigned pos = data[0];
    if (pos == 0) {
        return ReturnCode::OK;
    }

    data[0] = 0;

    if (pos >= data_length) {
        return ReturnCode::ERROR;
    }

    while (true) {
        uint8_t delta = data[pos];
        if (delta == 0) {
            break;
        }

        unsigned next_pos = pos + delta;

        data[pos] = magic_byte;

        pos = next_pos;

        if (pos >= data_length) {
            return ReturnCode::ERROR;
        }
    }

    data[pos] = magic_byte;

    return ReturnCode::OK;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
ReturnCode Deserializer<RxStream, Logger>::FindHeader_() {
    int res = read(rx_stream_, current_rx_buffer_, sizeof(kMagicByte));
    while (res != 0) {
        if (res < 0) {
            return ReturnCode::ERROR;
        }
        if (current_rx_buffer_->header.magic_byte == kMagicByte) {
            current_processed_length_ = sizeof(kMagicByte);
            return ReturnCode::OK;
        }
        lost_bytes_ += sizeof(kMagicByte);

        res = read(rx_stream_, current_rx_buffer_, sizeof(kMagicByte));
    }
    return ReturnCode::NO_DATA;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
ReturnCode Deserializer<RxStream, Logger>::ParseHeader_() {
    int res = read(rx_stream_,
                   reinterpret_cast<uint8_t *>(&current_rx_buffer_->header) +
                       current_processed_length_,
                   sizeof(MessageHeader) - current_processed_length_);
    if (res < 0) {
        return ReturnCode::ERROR;
    }
    current_processed_length_ += res;
    if (current_processed_length_ != sizeof(MessageHeader)) {
        return ReturnCode::NO_DATA;
    }
    if (current_header_->dest_address != self_address_) {
        current_processed_length_ = 0;
        return ReturnCode::FAIL;
    }
    return ReturnCode::OK;
}

template <concepts::stream::ByteReadableStreamConcept RxStream, typename Logger>
bool Deserializer<RxStream, Logger>::CheckCRC_() {
    uint8_t target_crc = crc::CountCRC8(reinterpret_cast<uint8_t *>(current_rx_buffer_),
                                        current_header_->length - kCRCLength);

    uint8_t current_crc = current_rx_buffer_->data_and_crc[current_header_->length -
                                                           sizeof(MessageHeader) - kCRCLength];

    return target_crc == current_crc;
}

}  // namespace hydrolib::bus::datalink