#include "quad_decoder.h"

/*
0	0	0	0	0	No change	No change	No movement
0	0	0	1	1	Increment	No change	Movement in positive direction
0	0	1	0	-1	Decrement	No change	Movement in negative direction
0	0	1	1	2	No change	Increment	Error: Double transition
0	1	0	0	-1	Decrement	No change	Movement in negative direction
0	1	0	1	0	No change	No change	No movement
0	1	1	0	2	No change	Increment	Error: Double transition
0	1	1	1	1	Increment	No change	Movement in positive direction
1	0	0	0	1	Increment	No change	Movement in positive direction
1	0	0	1	2	No change	Increment	Error: Double transition
1	0	1	0	0	No change	No change	No movement
1	0	1	1	-1	Decrement	No change	Movement in negative direction
1	1	0	0	2	No change	Increment	Error: Double transition
1	1	0	1	-1	Decrement	No change	Movement in negative direction
1	1	1	0	1	Increment	No change	Movement in positive direction
1	1	1	1	0	No change	No change	No movement
*/

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