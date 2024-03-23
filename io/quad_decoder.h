#pragma once

#include <stdint.h>
#include <stm32f10x_gpio.h>

class QuadDecoder {
 public:
  QuadDecoder(GPIO_TypeDef *port, uint16_t pin_a, uint16_t pin_b)
      : port_(port), pin_a_(pin_a), pin_b_(pin_b) {}

  int Update();

  int64_t position() const {
    return position_;
  }

  uint64_t errors() const {
    return errors_;
  }

 private:
  GPIO_TypeDef *port_;

  uint16_t pin_a_;
  uint16_t pin_b_;

  uint8_t prev_;

  int64_t position_;
  uint64_t errors_;
};