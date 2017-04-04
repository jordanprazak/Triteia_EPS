#ifndef EPS_PROTO_H
#define EPS_PROTO_H

#include <stdint.h>

struct iv_int {
  unsigned int iv;
  int16_t i;
  int16_t v;
};

// Set interrupt service routine for each latching current limiter
int16_t isr_latching_current_limiter();

// Set interrupt service routine for the UART
int16_t isr_UART();

#endif /* EPS_PROTO_H */
