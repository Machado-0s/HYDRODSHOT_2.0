#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "hydrolib_return_codes.hpp"

namespace hydrolib::ring_queue {

template <int CAPACITY>
class RingQueue {
 public:
  constexpr RingQueue() = default;

  void Clear();
  ReturnCode Drop(int number);

  ReturnCode PushByte(uint8_t byte);
  ReturnCode Push(const void *data, int data_length);
  ReturnCode PullByte(uint8_t *byte);
  ReturnCode Pull(void *data, int data_length);
  ReturnCode Read(void *data, int data_length, int shift) const;

  uint8_t &operator[](int index);
  const uint8_t &operator[](int index) const;

  [[nodiscard]] int GetLength() const;
  [[nodiscard]] int GetCapacity() const;
  [[nodiscard]] bool IsEmpty() const;
  [[nodiscard]] bool IsFull() const;

 private:
  std::array<uint8_t, CAPACITY + 1> buffer_ = {};

  int head_ = 0;
  int tail_ = 0;
};

template <int CAPACITY>
inline void RingQueue<CAPACITY>::Clear() {
  head_ = tail_;
}

template <int CAPACITY>
inline ReturnCode RingQueue<CAPACITY>::Drop(int number) {
  if (number > GetLength()) {
    return ReturnCode::FAIL;
  }
  head_ = (head_ + number) % CAPACITY;
  return ReturnCode::OK;
}

template <int CAPACITY>
inline ReturnCode RingQueue<CAPACITY>::PushByte(uint8_t byte) {
  if (IsFull()) {
    return ReturnCode::FAIL;
  }

  buffer_[tail_] = byte;
  tail_ = (tail_ + 1) % buffer_.size();

  return ReturnCode::OK;
}

template <int CAPACITY>
inline ReturnCode RingQueue<CAPACITY>::Push(const void *data, int data_length) {
  if (GetLength() + data_length > CAPACITY) {
    return ReturnCode::FAIL;
  }

  int forward_length = buffer_.size() - tail_;
  if (forward_length >= data_length) {
    memcpy(buffer_.data() + tail_, data, data_length);
  } else {
    memcpy(buffer_.data() + tail_, data, forward_length);
    memcpy(buffer_.data(),
           static_cast<const uint8_t *>(data) + forward_length,  // NOLINT
           data_length - forward_length);
  }
  tail_ = (tail_ + data_length) % buffer_.size();

  return ReturnCode::OK;
}

template <int CAPACITY>
inline ReturnCode RingQueue<CAPACITY>::PullByte(uint8_t *byte) {
  if (IsEmpty()) {
    return ReturnCode::FAIL;
  }

  *byte = buffer_[head_];
  head_ = (head_ + 1) % buffer_.size();

  return ReturnCode::OK;
}

template <int CAPACITY>
inline ReturnCode RingQueue<CAPACITY>::Pull(void *data, int data_length) {
  if (GetLength() < data_length) {
    return ReturnCode::FAIL;
  }

  int forward_length = buffer_.size() - head_;
  if (forward_length >= data_length) {
    memcpy(data, buffer_.data() + head_, data_length);
  } else {
    memcpy(data, buffer_.data() + head_, forward_length);
    memcpy(static_cast<uint8_t *>(data) + forward_length,  // NOLINT
           buffer_.data(), data_length - forward_length);
  }
  head_ = (head_ + data_length) % buffer_.size();

  return ReturnCode::OK;
}

template <int CAPACITY>
inline ReturnCode RingQueue<CAPACITY>::Read(void *data, int data_length,
                                            int shift) const {
  if (shift + data_length > GetLength()) {
    return ReturnCode::FAIL;
  }
  int forward_length = buffer_.size() - head_;
  if (forward_length > shift) {
    if (shift + data_length > forward_length) {
      memcpy(data, buffer_.data() + head_ + shift, forward_length - shift);
      memcpy(static_cast<uint8_t *>(data) + forward_length - shift,  // NOLINT
             buffer_.data(), data_length - (forward_length - shift));
    } else {
      memcpy(data, buffer_.data() + head_ + shift, data_length);
    }
  } else {
    memcpy(data, buffer_.data() + shift - forward_length, data_length);
  }
  return ReturnCode::OK;
}

template <int CAPACITY>
inline uint8_t &RingQueue<CAPACITY>::operator[](int index) {
  return buffer_[(head_ + index) % buffer_.size()];
}

template <int CAPACITY>
inline const uint8_t &RingQueue<CAPACITY>::operator[](int index) const {
  return buffer_[(head_ + index) % buffer_.size()];
}

template <int CAPACITY>
inline int RingQueue<CAPACITY>::GetLength() const {
  unsigned head = head_;
  if (tail_ >= head) {
    return tail_ - head;
  }
  return tail_ + buffer_.size() - head;
}

template <int CAPACITY>
inline int RingQueue<CAPACITY>::GetCapacity() const {
  return CAPACITY;
}

template <int CAPACITY>
inline bool RingQueue<CAPACITY>::IsEmpty() const {
  return head_ == tail_;
}

template <int CAPACITY>
inline bool RingQueue<CAPACITY>::IsFull() const {
  return head_ == (tail_ + 1) % buffer_.size();
}

}  // namespace hydrolib::ring_queue
