#include "hydrolib_ring_queue.hpp"
#include "stm32f4xx_hal.h"

#define MAGIC_BYTE 0x00
#define HEADER_SIZE 2
#define HEADER 0xFFFD

#define TOTAL_MOTOR_COUNT 14
#define DATA_SIZE TOTAL_MOTOR_COUNT
#define CRC_SIZE 1

#define PACKET_SIZE (HEADER_SIZE + DATA_SIZE + CRC_SIZE)
#define MAX_SCAN_BYTES 256

#define DSHOT_MIN_RPM -6000
#define DSHOT_MAX_RPM 6000
#define RX_BUFFER_SIZE 128

class ByteProtocol {
  public:
  ByteProtocol(UART_HandleTypeDef& huart, volatile float* buf) : huart_{huart}, buf_{buf} {}
  UART_HandleTypeDef& handle() { return huart_; }
  hydrolib::ReturnCode idleLineCallback(int size) { queue_.Push(huart_.pRxBuffPtr, size); }
  hydrolib::ReturnCode init();
  hydrolib::ReturnCode poll();

  private:
  UART_HandleTypeDef& huart_;
  hydrolib::ring_queue::RingQueue<2 * RX_BUFFER_SIZE> queue_;
  uint8_t Calculate_CRC8(const uint8_t* data, uint16_t length);
  uint8_t rxDataBuf[RX_BUFFER_SIZE] = {0};
  uint8_t packet_[PACKET_SIZE] = {0};
  volatile float* buf_;
  int shift_ = 0;
  bool packetFind_ = false;
};

hydrolib::ReturnCode ByteProtocol::init() {
  return static_cast<hydrolib::ReturnCode>(HAL_UARTEx_ReceiveToIdle_DMA(&huart_, rxDataBuf, sizeof(rxDataBuf)));
}

hydrolib::ReturnCode ByteProtocol::poll() {
  if (queue_.GetLength() >= PACKET_SIZE) {
    shift_ = 0;
    uint16_t header = 0;

    // Ищем заголовок в данных
    while (!packetFind_ && (queue_.GetLength() - shift_ >= PACKET_SIZE)) {
      uint8_t header_bytes[HEADER_SIZE];

      // Читаем два байта для проверки заголовка
      if (queue_.Read(header_bytes, HEADER_SIZE, shift_) != hydrolib::ReturnCode::OK) {
        break;
      }
      header = (header_bytes[0] << 8) | header_bytes[1];

      if (header == HEADER) {
        if (queue_.Read(packet_, PACKET_SIZE, shift_) != hydrolib::ReturnCode::OK) {
          break;
        }

        uint8_t crc_calculated = Calculate_CRC8(&packet_[HEADER_SIZE], DATA_SIZE);
        uint8_t crc_received = packet_[HEADER_SIZE + DATA_SIZE];

        if (crc_calculated == crc_received && buf_ != nullptr) {
          for (int i = 0; i < TOTAL_MOTOR_COUNT; i++) {
            uint8_t byte_val = packet_[HEADER_SIZE + i];

            if (byte_val >= 100 && byte_val <= 200) {
              int32_t signed_val = (int32_t)byte_val - 150;  // -50..+50
              buf_[i] = (signed_val * DSHOT_MAX_RPM) / 50.0f;
            }
          }

          packetFind_ = true;
          return hydrolib::ReturnCode::OK;
        } else {
          queue_.Drop(1);
          shift_++;
        }
      } else {
        queue_.Drop(1);
        shift_++;
      }
    }
  }

  return hydrolib::ReturnCode::NO_DATA;
}

uint8_t ByteProtocol::Calculate_CRC8(const uint8_t* data, uint16_t length) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
