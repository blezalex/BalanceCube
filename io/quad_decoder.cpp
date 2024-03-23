#include "quad_decoder.h"

static constexpr int8_t kTransitionTable[16] = { 0, 1, -1, 2, -1, 0, 2, 1, 1, 2, 0, -1, 2, -1, 1, 0 };

int QuadDecoder::Update(){
  uint8_t a = GPIO_ReadInputDataBit(port_, pin_a_) ? 1 : 0;
  uint8_t b = GPIO_ReadInputDataBit(port_, pin_b_) ? 1 : 0;

  uint8_t cur = a << 1 | b;
  int8_t result = kTransitionTable[prev_ << 2 | cur];
  if (result == 2) {
    errors_++;
  }
  else {
    position_ += result;
  }
  prev_ = cur;
  return result;
}