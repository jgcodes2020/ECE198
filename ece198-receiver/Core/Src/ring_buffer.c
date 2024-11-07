/*
 * ring_buffer.c
 *
 *  Created on: Nov 7, 2024
 *      Author: jgcodes
 */

#include "ring_buffer.h"

void ring_buffer_init(RingBuffer* self) {
  memset(self, 0, sizeof(RingBuffer));
}
extern bool ring_buffer_put(RingBuffer* self, uint8_t b) {
  if ((self->head + 1) == self->tail) {
    return false;
  }
  self->data[self->head++] = b;
  return true;
}
extern TakeResult ring_buffer_take(RingBuffer* self) {
  if (self->tail == self->head) {
    return (TakeResult) {
      .is_valid = false,
      .value = 0
    };
  }
  uint8_t result = self->data[self->tail++];
  return (TakeResult) {
    .is_valid = true,
    .value = result
  };
}
