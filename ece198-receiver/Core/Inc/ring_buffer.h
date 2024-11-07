/*
 * ring_buffer.h
 *
 *  Created on: Nov 6, 2024
 *      Author: jgcodes
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdbool.h>
#include <stdint.h>

// this is conveniently enough for an 8-bit index to simply overflow
#define RING_LEN 256

typedef struct RingBuffer {
  volatile uint8_t data[256];
  volatile uint8_t head;
  volatile uint8_t tail;
} RingBuffer;
typedef struct TakeResult {
  bool is_valid;
  uint8_t value;
} TakeResult;

extern void ring_buffer_init(RingBuffer* self);
extern bool ring_buffer_put(RingBuffer* self, uint8_t b);
extern TakeResult ring_buffer_take(RingBuffer* self);

#endif /* INC_RING_BUFFER_H_ */
